#!/usr/bin/env python
'''
refer to carla/PythonAPI/examples/visualize_multiple_sensors.py
'''
import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import argparse
import random
import time, math
import weakref
import collections
import numpy as np
import queue, threading, struct
from datetime import datetime

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
    
from helper.carla_utils import get_actor_blueprints, find_weather_presets, get_actor_display_name, get_nearest_spawn_point
from utils import plogging
from utils import udpwrapper

global g_logger
g_logger = plogging.get_logger()

class CustomTimer(object):
    def __init__(self):
        try:
            self.timer = time.perf_counter
        except AttributeError:
            self.timer = time.time

    def time(self):
        return self.timer()


class SensorWrapperBase(object):
    def __init__(self, world, display_man, display_pos, sensor_type, idx=-1):
        self.surface = None
        self.world = world
        self.display_man = display_man
        self.display_pos = display_pos
        self.timer = CustomTimer()
        self.time_processing = 0.0
        self.tics_processing = 0
        self.sensor = None
        self.t_start = self.timer.time()
        self.sensor_type = sensor_type
        self.idx = idx
        self.last_frame_time = time.time()
        self.msg_queue = queue.Queue(maxsize=10)
        self.msg_thread = threading.Thread(target=self.process_msg, args=())
        self.msg_thread.setDaemon(True)
        self.msg_thread.start()

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)
            self.surface = None

    def process_msg(self):
        g_logger.error("this is sensor[%d] base class stub, quit!", self.idx)
        pass

    def destroy(self):
        if self.sensor:
            self.sensor.destroy()
            self.sensor = None
        dt = self.timer.time() - self.t_start
        g_logger.info("[%s] tics: %d, dt: %f, fps: %.2f", self.sensor_type, self.tics_processing, dt, self.tics_processing/dt)

    def __del__(self):
        if self.sensor:
            self.destroy()


class RadarWrapper(SensorWrapperBase):
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, idx=-1):
        SensorWrapperBase.__init__(self, world, display_man, display_pos, sensor_type, idx)
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == "Radar":
            radar_bp = self.world.get_blueprint_library().find('sensor.other.radar')
            for key in sensor_options:
                radar_bp.set_attribute(key, sensor_options[key])

            radar = self.world.spawn_actor(radar_bp, transform, attach_to=attached)
            radar.listen(self.save_radar_image)
            return radar
        else:
            return None

    def save_radar_image(self, radar_data):
        points_2d = np.zeros(shape=(len(radar_data), 2), dtype=np.float)
        disp_size = self.display_man.get_display_size()
        radar_range = float(self.sensor_options['range'])
        radar_rot = radar_data.transform.rotation
        # g_logger.info("radar_range: {}, point num: {}, disp_size: {}, transform: {}".format(radar_range, len(radar_data), disp_size, radar_data.transform))
        # carla.RadarDetection
        for idx, detect in enumerate(radar_data):
            dist = detect.depth
            if dist > radar_range:
                pass
                # g_logger.warn("radar dist exceed range: {}".format(dist))
                # dist = radar_range            
            # image left is forward looking
            vec_3d = carla.Vector3D(
                x=dist*math.cos(detect.altitude)*math.cos(detect.azimuth),
                y=dist*math.cos(detect.altitude)*math.sin(detect.azimuth),
                z=dist*math.sin(detect.altitude))
            carla.Transform(carla.Location(), radar_rot).transform(vec_3d)

            points_2d[idx, 0] = vec_3d.x
            points_2d[idx, 1] = vec_3d.y
            points_2d[idx, :] *= min(disp_size) / 2.0 / radar_range
            points_2d[idx, :] += (0.5 * disp_size[0], 0.5 * disp_size[1])
            # print("point-{}: {}, {}".format(idx, points_2d[idx, 0], points_2d[idx, 1]))

        radar_data = points_2d[:, :2]
        radar_data = np.fabs(radar_data)  # pylint: disable=E1111
        radar_data = radar_data.astype(np.int32)
        radar_data = np.reshape(radar_data, (-1, 2))
        radar_img_size = (disp_size[0], disp_size[1], 3)
        radar_img = np.zeros((radar_img_size), dtype=np.uint8)

        try:
            radar_img[tuple(radar_data.T)] = (255, 255, 64)
        except IndexError:
            pass

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(radar_img)
        self.tics_processing += 1
        if self.tics_processing % 100 == 0:
            fps = 100 / (time.time() - self.last_frame_time)
            self.last_frame_time = time.time()
            g_logger.info("[%s] frame: %d, fps: %.2f", self.sensor_type, self.tics_processing, fps)


class LidarWrapper(SensorWrapperBase):
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, idx=-1):
        SensorWrapperBase.__init__(self, world, display_man, display_pos, sensor_type, idx)
        self.sensor_options = sensor_options
        self.display_man.add_sensor(self)
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.discard_counter = 0
        self.CH128X1_FRAME_SIZE = 1206  # msop frame contain 171 points, and 9 bytes additional info
        self.CH128X1_SUBFRAME_BYTES = 7
        self.CH128X1_SUBFRAME_POINTS = 171
        self.CH128X1_MSOP_HEADER = bytearray([0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11])
        self.CH128X1_DIFOP_HEADER = bytearray([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
        self.CH128X1_DIFOP_TAIL = bytearray([0x0F, 0xF0])
        self.CH128X1_UCWP_HEADER = bytearray([0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA])
        self.CH128X1_UCWP_TAIL = bytearray([0x0F, 0xF0])
        self.msop_frame = bytearray(self.CH128X1_FRAME_SIZE)
        # msop data send from 2369 to 2368
        self.msop_udp_client = udpwrapper.SocketClient(host="127.0.0.1", port=12368+self.idx)
        # difop data send from 2368 to 2369 
        self.difop_udp_client = udpwrapper.SocketClient(host="127.0.0.1", port=12369+self.idx)
        # ucwp data comes into 2368 from 2369, but in case lidar and PC are on the same device, we change ucwp port to 22368
        self.ucwp_udp_server = udpwrapper.SocketServer(host="127.0.0.1", port=22368+self.idx, len=self.CH128X1_FRAME_SIZE)
        self.ucwp_udp_server.start_recv_thread(run_background=True)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            # lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])
            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            self.lower_fov = float(sensor_options["lower_fov"])
            self.upper_fov = float(sensor_options["upper_fov"])
            self.channels = float(sensor_options["channels"])
            self.vertical_resolution = (self.upper_fov - self.lower_fov) / self.channels
            self.range = float(self.sensor_options['range'])
            g_logger.info("lidar[%d] channels: %d, vertical_resolution: %.2f", self.idx, self.channels, self.vertical_resolution)

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
            lidar.listen(self.save_lidar_image)
            return lidar
        else:
            return None

    '''
    # make timestamp from host now() or carla timestamp
    '''
    def make_timestamp(self, timestamp=time.time()):
        # year, month, day, hour, min, sec, microsec
        # ['2023', '11', '07', '14', '49', '16', '378058']
        dt = datetime.fromtimestamp(timestamp)
        return dt.strftime("%Y,%m,%d,%H,%M,%S,%f").split(",")

    '''
    # make lidar protocol following Leihen-CH128X1, 120*25deg, 1520000pps, 88.7Mbps
    '''
    def process_msg(self):
        g_logger.info("lidar[%d] msg thread start.", self.idx)
        msop_counter = 0
        last_msop_time = time.time()
        while True:
            image = self.msg_queue.get(block=True, timeout=None)
            if image is None:
                g_logger.error("lidar[%d] msg thread recv None, quit.", self.idx)
                break
            # process image
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            points_count = int(points.shape[0])
            subframe_count = int(points_count / self.CH128X1_SUBFRAME_POINTS) # the max is 8983

            for j in range(subframe_count):
                for i in range(self.CH128X1_SUBFRAME_POINTS):
                    # sub frame per point
                    sub_frame = bytearray(self.CH128X1_SUBFRAME_BYTES)
                    p = points[i + j*self.CH128X1_SUBFRAME_POINTS]
                    fx = p[0]
                    fy = p[1]
                    fz = p[2]
                    fi = p[3]
                    # distance, float32
                    dist = math.sqrt(fx**2 + fy**2)
                    # horizontal angle, float32
                    hori_angle = math.acos(fy/dist)*57.3
                    # vertical angle, float32
                    vert_angle = math.atan(fz/dist)*57.3
                    # line numer, int32
                    line_num = round((vert_angle - self.lower_fov) / self.vertical_resolution)
                    sub_frame[0] = line_num
                    # horizontal angle, resolution 0.01 degree
                    sub_frame[1] = int(hori_angle)
                    sub_frame[2] = int(100*(hori_angle - int(hori_angle)))
                    # distance, resolution 1/256 cm
                    dist_cm = int(dist*100)
                    dist_residual = int(256*(dist*100 - dist_cm))
                    sub_frame[3:5] = struct.pack(">H", int(dist_cm))
                    sub_frame[5] = dist_residual
                    # intensity, 0~255
                    sub_frame[6] = int(255*fi)
                    self.msop_frame[i*self.CH128X1_SUBFRAME_BYTES : (i+1)*self.CH128X1_SUBFRAME_BYTES] = sub_frame
                ts = self.make_timestamp(time.time())
                # self.msop_frame[1197, 1198, 1199] = uint8(hr, min, sec)
                self.msop_frame[1197] = int(ts[3])
                self.msop_frame[1198] = int(ts[4])
                self.msop_frame[1199] = int(ts[5])
                # self.msop_frame[1200, 1201, 1202, 1203] = uint32(microsec)
                self.msop_frame[1200:1204] = struct.pack(">I", int(ts[6]))
                # self.msop_frame[1204] = 0x80  stands for CH128X1
                self.msop_frame[1204] = 0x80
                # self.msop_frame[1205] = 0x02  stands for double return
                self.msop_frame[1205] = 0x02

                # indicator of the start of pointcloud, could be anywhere in the msop frame
                if j == 0:
                    self.msop_frame[0:7] = self.CH128X1_MSOP_HEADER
                self.msop_udp_client.send_array(self.msop_frame)

            msop_counter += 1
            # g_logger.info("lidar[%d] image [%d] timestamp: %.6f, channels: %d, vert_res: %.2f", self.idx, image.frame, image.timestamp, image.channels, self.vertical_resolution)
            if msop_counter % 10 == 0:
                fps = 10 / (time.time() - last_msop_time)
                last_msop_time = time.time()
                g_logger.info("lidar[%d] msg thread process msop_counter: %d, fps: %.2f", self.idx, msop_counter, fps)

                # send difop frame
                difop_frame = bytearray(self.CH128X1_FRAME_SIZE)
                difop_frame[0:8] = self.CH128X1_DIFOP_HEADER
                ts = self.make_timestamp(time.time())
                difop_frame[52] = (int(ts[0]) - 2000) % 255 # year 0~255
                difop_frame[53] = int(ts[1]) # month 1~12
                difop_frame[54] = int(ts[2]) # day 1~31
                difop_frame[55] = int(ts[3]) # hour 0~23
                difop_frame[56] = int(ts[4]) # min  0~59
                difop_frame[57] = int(ts[5]) # sec  0~59
                difop_frame[1204:1206] = self.CH128X1_DIFOP_TAIL

                self.difop_udp_client.send_array(difop_frame)

        g_logger.info("lidar[%d] msg thread exit.", self.idx)

    def save_lidar_image(self, image):
        t_start = self.timer.time()
        try:
            # msop frame encoding is about 3sec per frame, just put slow
            if self.tics_processing % 20 == 0:
            # if not self.msg_queue.full():
                self.msg_queue.put(image, block=False, timeout=None)
        except queue.Full:
            self.discard_counter += 1
            if self.discard_counter % 100 == 0:
                g_logger.warn("lidar[%d] queue full, discard frame count: %d", self.idx, self.discard_counter)

        disp_size = self.display_man.get_display_size()
        lidar_rot = image.transform.rotation
        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        # g_logger.info("lidar points count: %d", int(points.shape[0]))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        # fps too slow if transform all lidar points, so we down-sample
        downsample_size = int(points.shape[0]/20-1)
        points_3d = np.zeros(shape=(downsample_size, 3), dtype=np.float)
        for idx in range(downsample_size):
            point = points[idx*20]
            vec_3d = carla.Vector3D(
                x=float(point[0]),
                y=float(point[1]),
                z=float(point[2]))
            carla.Transform(carla.Location(), lidar_rot).transform(vec_3d)
            points_3d[idx, 0] = vec_3d.x
            points_3d[idx, 1] = vec_3d.y
            points_3d[idx, 2] = vec_3d.z

        lidar_data = np.array(points_3d[:, :2])
        lidar_data *= min(disp_size) / (2*self.range)
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (64, 255, 64)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        if self.tics_processing % 100 == 0:
            fps = 100 / (time.time() - self.last_frame_time)
            self.last_frame_time = time.time()
            g_logger.info("[%s] frame: %d, fps: %.2f", self.sensor_type, self.tics_processing, fps)


class CameraWrapper(SensorWrapperBase):
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos, idx=-1):
        SensorWrapperBase.__init__(self, world, display_man, display_pos, sensor_type, idx)
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'RGBCamera':
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
            disp_size = self.display_man.get_display_size()
            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))
            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_rgb_image)
            return camera
        elif sensor_type == "SemanticCamera":
            camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
            disp_size = self.display_man.get_display_size()
            for key in sensor_options:
                camera_bp.set_attribute(key, sensor_options[key])
            camera_bp.set_attribute('image_size_x', str(disp_size[0]))
            camera_bp.set_attribute('image_size_y', str(disp_size[1]))
            camera = self.world.spawn_actor(camera_bp, transform, attach_to=attached)
            camera.listen(self.save_semantic_image)
            return camera
        else:
            return None

    def save_rgb_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.Raw)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1
        if self.tics_processing % 100 == 0:
            fps = 100 / (time.time() - self.last_frame_time)
            self.last_frame_time = time.time()
            g_logger.info("[%s] frame: %d, fps: %.2f", self.sensor_type, self.tics_processing, fps)


    def save_semantic_image(self, image):
        t_start = self.timer.time()

        image.convert(carla.ColorConverter.CityScapesPalette)
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1


# ==============================================================================
# -- CollisionSensor -----------------------------------------------------------
# ==============================================================================

class CollisionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None
        self.history = []
        self._parent = parent_actor
        self.hud = hud
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.collision')
        self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: CollisionSensor._on_collision(weak_self, event))

    def get_collision_history(self):
        history = collections.defaultdict(int)
        for frame, intensity in self.history:
            history[frame] += intensity
        return history

    @staticmethod
    def _on_collision(weak_self, event):
        self = weak_self()
        if not self:
            return
        actor_type = get_actor_display_name(event.other_actor)
        self.hud.notification('Collision with %r' % actor_type)
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame, intensity))
        if len(self.history) > 4000:
            self.history.pop(0)


# ==============================================================================
# -- LaneInvasionSensor --------------------------------------------------------
# ==============================================================================

class LaneInvasionSensor(object):
    def __init__(self, parent_actor, hud):
        self.sensor = None

        # If the spawn object is not a vehicle, we cannot use the Lane Invasion Sensor
        if parent_actor.type_id.startswith("vehicle."):
            self._parent = parent_actor
            self.hud = hud
            world = self._parent.get_world()
            bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
            self.sensor = world.spawn_actor(bp, carla.Transform(), attach_to=self._parent)
            # We need to pass the lambda a weak reference to self to avoid circular
            # reference.
            weak_self = weakref.ref(self)
            self.sensor.listen(lambda event: LaneInvasionSensor._on_invasion(weak_self, event))

    @staticmethod
    def _on_invasion(weak_self, event):
        self = weak_self()
        if not self:
            return
        lane_types = set(x.type for x in event.crossed_lane_markings)
        text = ['%r' % str(x).split()[-1] for x in lane_types]
        self.hud.notification('Crossed line %s' % ' and '.join(text))

# ==============================================================================
# -- GnssSensor ----------------------------------------------------------------
# ==============================================================================

class GnssSensor(object):
    def __init__(self, parent_actor, location=carla.Location()):
        self.sensor = None
        self._parent = parent_actor
        self.lat = 0.0
        self.lon = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.gnss')
        # set gnss onto midpoint of rear axle
        self.sensor = world.spawn_actor(bp, carla.Transform(location, carla.Rotation()), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda event: GnssSensor._on_gnss_event(weak_self, event))

    @staticmethod
    def _on_gnss_event(weak_self, event):
        self = weak_self()
        if not self:
            return
        self.lat = event.latitude
        self.lon = event.longitude


# ==============================================================================
# -- IMUSensor -----------------------------------------------------------------
# ==============================================================================


class IMUSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        self.accelerometer = (0.0, 0.0, 0.0)
        self.gyroscope = (0.0, 0.0, 0.0)
        self.compass = 0.0
        world = self._parent.get_world()
        bp = world.get_blueprint_library().find('sensor.other.imu')
        self.sensor = world.spawn_actor(
            bp, carla.Transform(), attach_to=self._parent)
        # We need to pass the lambda a weak reference to self to avoid circular
        # reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda sensor_data: IMUSensor._IMU_callback(weak_self, sensor_data))

    @staticmethod
    def _IMU_callback(weak_self, sensor_data):
        self = weak_self()
        if not self:
            return
        limits = (-99.9, 99.9)
        self.accelerometer = (
            max(limits[0], min(limits[1], sensor_data.accelerometer.x)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.y)),
            max(limits[0], min(limits[1], sensor_data.accelerometer.z)))
        self.gyroscope = (
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.x))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.y))),
            max(limits[0], min(limits[1], math.degrees(sensor_data.gyroscope.z))))
        self.compass = math.degrees(sensor_data.compass)
