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

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
    
from helper.carla_utils import get_actor_blueprints, find_weather_presets, get_actor_display_name, get_nearest_spawn_point
from utils import plogging
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
    def __init__(self, world, display_man, display_pos, sensor_type):
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

    def render(self):
        if self.surface is not None:
            offset = self.display_man.get_display_offset(self.display_pos)
            self.display_man.display.blit(self.surface, offset)

    def destroy(self):
        if self.sensor:
            self.sensor.destroy()
            self.sensor = None
        dt = self.timer.time() - self.t_start
        g_logger.info("[%s] tics: %d, dt: %f, fps: %f", self.sensor_type, self.tics_processing, dt, self.tics_processing/dt)

    def __del__(self):
        if self.sensor:
            self.destroy()


class RadarWrapper(SensorWrapperBase):
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        SensorWrapperBase.__init__(self, world, display_man, display_pos, sensor_type)
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

        radar_img[tuple(radar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(radar_img)
        self.tics_processing += 1


class LidarWrapper(SensorWrapperBase):
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        SensorWrapperBase.__init__(self, world, display_man, display_pos, sensor_type)
        self.sensor = self.init_sensor(sensor_type, transform, attached, sensor_options)
        self.sensor_options = sensor_options
        self.display_man.add_sensor(self)

    def init_sensor(self, sensor_type, transform, attached, sensor_options):
        if sensor_type == 'LiDAR':
            lidar_bp = self.world.get_blueprint_library().find('sensor.lidar.ray_cast')
            lidar_bp.set_attribute('range', '100')
            lidar_bp.set_attribute('dropoff_general_rate', lidar_bp.get_attribute('dropoff_general_rate').recommended_values[0])
            lidar_bp.set_attribute('dropoff_intensity_limit', lidar_bp.get_attribute('dropoff_intensity_limit').recommended_values[0])
            lidar_bp.set_attribute('dropoff_zero_intensity', lidar_bp.get_attribute('dropoff_zero_intensity').recommended_values[0])
            for key in sensor_options:
                lidar_bp.set_attribute(key, sensor_options[key])

            lidar = self.world.spawn_actor(lidar_bp, transform, attach_to=attached)
            lidar.listen(self.save_lidar_image)
            return lidar
        else:
            return None

    def save_lidar_image(self, image):
        t_start = self.timer.time()

        disp_size = self.display_man.get_display_size()
        lidar_range = 2.0*float(self.sensor_options['range'])
        lidar_rot = image.transform.rotation

        points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        points_3d = np.zeros(shape=(points.shape[0], 3), dtype=np.float)
        for idx, point in enumerate(points):
            vec_3d = carla.Vector3D(
                x=float(point[0]),
                y=float(point[1]),
                z=float(point[2]))
            carla.Transform(carla.Location(), lidar_rot).transform(vec_3d)
            points_3d[idx, 0] = vec_3d.x
            points_3d[idx, 1] = vec_3d.y
            points_3d[idx, 2] = vec_3d.z

        lidar_data = np.array(points_3d[:, :2])
        lidar_data *= min(disp_size) / lidar_range
        lidar_data += (0.5 * disp_size[0], 0.5 * disp_size[1])
        lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
        lidar_data = lidar_data.astype(np.int32)
        lidar_data = np.reshape(lidar_data, (-1, 2))
        lidar_img_size = (disp_size[0], disp_size[1], 3)
        lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)

        lidar_img[tuple(lidar_data.T)] = (255, 255, 255)

        if self.display_man.render_enabled():
            self.surface = pygame.surfarray.make_surface(lidar_img)

        t_end = self.timer.time()
        self.time_processing += (t_end-t_start)
        self.tics_processing += 1


class CameraWrapper(SensorWrapperBase):
    def __init__(self, world, display_man, sensor_type, transform, attached, sensor_options, display_pos):
        SensorWrapperBase.__init__(self, world, display_man, display_pos, sensor_type)
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
