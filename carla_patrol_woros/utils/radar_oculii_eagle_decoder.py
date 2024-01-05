#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, socket, struct, queue, math
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
# apt install ros-foxy-sensor-msgs-py
from sensor_msgs_py import point_cloud2
import numpy as np
import udpwrapper
import plogging

global g_logger

class Ros2BridgeNode(Node):
    def __init__(self):
        super().__init__("Ros2BridgeNode")
        self.pub = self.create_publisher(PointCloud2, "radar_oculii_eagle", 10)
        self.header = Header()
        ## set rviz fixed-frame to ch128x1
        self.header.frame_id = "oculii_eagle"
        self.dtype = PointField.FLOAT32
        self.max_points = 1400
        self.fields = [PointField(name='x', offset=0, datatype=self.dtype, count=1),
                       PointField(name='y', offset=4, datatype=self.dtype, count=1),
                       PointField(name='z', offset=8, datatype=self.dtype, count=1),
                       PointField(name='intensity', offset=12, datatype=self.dtype, count=1)]
        # self.np_points = np.empty(shape=[0,4], dtype=np.float32)
        self.np_points = np.empty(shape=[self.max_points+1,4], dtype=np.float32)
        self.cloud_counter = 0
        self.points_counter = 0

    def add_point(self, raw_sub_frame, frame_finish=False):
        ### decode one raw_sub_frame to one point
        if frame_finish:
            ts = time.time()
            # g_logger.warning("publish pointcloud ts: %f, %s", ts, str(tdate))
            rclpy_ts = rclpy.time.Time(seconds=int(ts), nanoseconds=(ts-int(ts))*1e9)
            self.header.stamp = rclpy_ts.to_msg()
            msg = point_cloud2.create_cloud(self.header, self.fields, self.np_points)
            self.pub.publish(msg)
            self.cloud_counter += 1
            g_logger.warning("publish pointcloud[%d] with %d points", self.cloud_counter, msg.width)
            # clear pointcloud
            # self.np_points = np.empty(shape=[0,4], dtype=np.float32)
            self.points_counter = 0
        else:
            return
            vert_angle = (raw_sub_frame[0]*self.vertical_resolution + self.lower_fov)/57.3
            # g_logger.info("line_num: %d, vert_angle: %.2f", raw_sub_frame[0], vert_angle*57.3)
            # hori_angle = (raw_sub_frame[1] + raw_sub_frame[2]*0.01)/57.3
            hori_angle = 0.01*(struct.unpack(">H", raw_sub_frame[1:3])[0])/57.3
            dist = 0.01*(struct.unpack(">H", raw_sub_frame[3:5])[0]) + 0.01*raw_sub_frame[5]/256
            # append new point to cloud
            fx = dist * math.sin(hori_angle)
            fy = dist * math.cos(hori_angle)
            fz = dist * math.tan(vert_angle)
            fi = raw_sub_frame[6] / 255.0

            ## too slow if we append one point each loop, numpy memory must be pre-allocated
            # self.np_points = np.append(self.np_points, [[fx, fy, fz, fi]], axis=0)
            self.np_points[self.points_counter][0] = fx
            self.np_points[self.points_counter][1] = fy
            self.np_points[self.points_counter][2] = fz
            self.np_points[self.points_counter][3] = fi
            self.points_counter += 1
            # if self.points_counter % 10000 == 0:
            #     g_logger.info("decode point [%d] x: %.2f, y: %.2f, z: %.2f, i: %.2f", self.points_counter, fx, fy, fz, fi)


class SocketServerOculii(udpwrapper.SocketServer):
    def __init__(self, host="127.0.0.1", port=19910, len=1200):
        super(SocketServerOculii, self).__init__(host=host, port=port, len=len)
        self.OCULII_MIN_AZIMUTH = -56.5
        self.OCULII_MAX_AZIMUTH = 56.5
        self.OCULII_MIN_ELEVATION = -22.5
        self.OCULII_MAX_ELEVATION = 22.5
        self.OCULII_HANDSHAKE_SIZE = 24
        self.OCULII_HANDSHAKE_MAGIC = bytearray([0x01, 0x09, 0x08, 0x09, 0x01, 0x00, 0x02, 0x02])
        self.OCULII_HEADER_SIZE = 48
        self.OCULII_HEADER_MAGIC = bytearray([0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07])
        self.HEADER_RANGE_ACCURACY = 0.43 #m
        self.HEADER_DOPPLER_ACCURACY = 0.09 #m/s
        self.HEADER_AZIMUTH_ACCURACY = 0.44 #deg
        self.HEADER_ELEVATION_ACCURACY = 0.175 #deg
        self.OCULII_VERSION = 12300
        self.OCULII_DETECTION_BLOCK_SIZE = 8
        self.OCULII_TRACK_BLOCK_SIZE = 32
        self.OCULII_FOOTER_SIZE = 32
        self.OCULII_FREQ_HZ = 15
        self.OCULII_CYCLE_SEC = 1.0 / self.OCULII_FREQ_HZ
        self.OCULII_MAX_POINTS_PER_SCAN = 1400 # ARS430:640
        self.OCULII_MAX_TRACK_PER_SCAN = 128   # ARS430:62
        self.FRAME_MAX_SIZE = self.OCULII_HEADER_SIZE + \
                              self.OCULII_DETECTION_BLOCK_SIZE*self.OCULII_MAX_POINTS_PER_SCAN + \
                              self.OCULII_TRACK_BLOCK_SIZE*self.OCULII_MAX_TRACK_PER_SCAN + \
                              self.OCULII_FOOTER_SIZE # 15376 bytes
        self.SUB_FRAME_SIZE = 1200 # bytes, totally 16 sub-frames

        self.ros_bridge = Ros2BridgeNode()
        self.full_frame = bytearray(0)
        self.wait_full_frame_size = -1
        self.frame_queue = queue.Queue(maxsize=1000)
        self.frame_finish = False
        self.handshake_counter = 0
        self.full_frame_counter = 0
        self.discard_counter = 0
        self.decoder_thread = threading.Thread(target=self.decode_frame, args=())
        self.decoder_thread.setDaemon(True)
        self.decoder_thread.start()

    def parse_handshake(self, handshake):
        if handshake[0:8] == self.OCULII_HANDSHAKE_MAGIC:
            full_frame_size = struct.unpack("<I", handshake[8:12])[0]
            b4nano = struct.unpack("<I", handshake[12:16])[0]
            b4sec = struct.unpack("<I", handshake[16:20])[0]
            g_logger.info("oculii handshake[%d], time %d:%d, full_frame_size: %d", self.handshake_counter, b4sec, b4nano, full_frame_size)
            self.handshake_counter += 1
            self.wait_full_frame_size = full_frame_size
            return self.wait_full_frame_size
        else:
            return -1

    def post_process(self, msg):
        # the msg passed in is not used
        oculii_frame = self.msg
        if not oculii_frame:
            return
        # g_logger.info("oculii_frame size: %d", len(oculii_frame))
        if len(oculii_frame) == self.OCULII_HANDSHAKE_SIZE:
            if self.parse_handshake(oculii_frame) > 0:
                self.full_frame = bytearray(0)
                return 0

        if len(self.full_frame) < self.wait_full_frame_size:
            self.full_frame += oculii_frame
            # g_logger.info("oculii_frame size: %d, full_frame size: %d, wait_full_frame_size: %d", len(oculii_frame), len(self.full_frame), self.wait_full_frame_size)
        if len(self.full_frame) == self.wait_full_frame_size:
            self.wait_full_frame_size = -1
            try:
                # if not self.msg_queue.full():
                self.frame_queue.put(self.full_frame, block=False, timeout=None)
                g_logger.info("push full_frame [%d] to queue, full_frame_size: %d", self.full_frame_counter, len(self.full_frame))
                self.full_frame_counter += 1
            except queue.Full:
                self.discard_counter += 1
                if self.discard_counter % 1 == 0:
                    g_logger.error("frame queue full, discard frame count: %d", self.discard_counter)
        elif len(self.full_frame) > self.wait_full_frame_size:
            g_logger.error("full_frame size %d overflow", len(self.full_frame))

        return 1

    def decode_frame(self):
        g_logger.info("oculii decoder thread start.")
        frame_counter = 0
        while True:
            oculii_frame = self.frame_queue.get(block=True, timeout=None)
            frame_header = oculii_frame[0:48]
            if frame_header[0:8] != self.OCULII_HEADER_MAGIC:
                g_logger.error("wrong header magic")
                continue
            frame_counter = struct.unpack("<I", frame_header[8:12])[0]
            firm_version = struct.unpack("<I", frame_header[12:16])[0]
            det_count = struct.unpack("<H", frame_header[16:18])[0]
            track_count = struct.unpack("<H", frame_header[18:20])[0]
            g_logger.info("frame_counter [%d], firm_version: %d, det_count: %d, track_count: %d", frame_counter, firm_version, det_count, track_count)

            range_accuracy = struct.unpack("<H", frame_header[32:34])[0] / 10000.0
            doppler_accuracy = struct.unpack("<H", frame_header[34:36])[0] / 10000.0
            azimuth_accuracy = struct.unpack("<H", frame_header[36:38])[0] / 10000.0
            elevation_accuracy = struct.unpack("<H", frame_header[38:40])[0] / 10000.0
            g_logger.info("frame_counter [%d], range_accuracy: %.3f, doppler_accuracy: %.3f, azimuth_accuracy: %.3f, elevation_accuracy: %.3f", frame_counter, range_accuracy, doppler_accuracy, azimuth_accuracy, elevation_accuracy)

            ### parse detections
            block_start_idx = self.OCULII_HEADER_SIZE
            for i in range(det_count):
                block_detection = oculii_frame[block_start_idx:(block_start_idx+self.OCULII_DETECTION_BLOCK_SIZE)]
                block_start_idx += self.OCULII_DETECTION_BLOCK_SIZE
                self.ros_bridge.add_point(raw_sub_frame = block_detection, frame_finish = (i==(det_count-1)))

        g_logger.info("oculii decoder thread exit.")

if __name__ == "__main__":
    plogging.init_logger(log_dir="./", file_name="radar_oculii_eagle_decoder")
    g_logger = plogging.get_logger()
    rclpy.init(args=None)

    radar1_frame_receiver = SocketServerOculii(host="127.0.0.1", port=19910, len=1200)
    radar1_frame_receiver.start_recv_thread(run_background=True)
    counter = 0
    while True:
        time.sleep(5.0)
        counter += 1
        g_logger.info("heartbeat counter: %d", counter)
