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
global g_date # lidar timestamp
global g_motor_speed # motor speed, 0x04B0=1200rpm, 0x0258=600rpm, 0x012C=300rpm

class Ros2BridgeNode(Node):
    def __init__(self):
        super().__init__("Ros2BridgeNode")
        self.max_points = 153600
        self.lower_fov = -18.0
        self.upper_fov = 7.0
        self.channels = 128
        self.vertical_resolution = (self.upper_fov - self.lower_fov) / self.channels
        self.range = 200.0
        self.pub = self.create_publisher(PointCloud2, "lidar_ch128x1", 10)
        self.header = Header()
        ## set rviz fixed-frame to ch128x1
        self.header.frame_id = "ch128x1"
        self.dtype = PointField.FLOAT32
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
        if frame_finish or self.points_counter > self.max_points:
            if self.points_counter < 10:
                g_logger.warning("abnormal small pointcloud %d points, discard", self.points_counter)
            else:
                if self.points_counter > self.max_points:
                    g_logger.warning("abnormal large pointcloud %d points", self.points_counter)
                tdate = datetime(year=g_date[0], month=g_date[1], day=g_date[2], hour=g_date[3], minute=g_date[4], second=g_date[5], microsecond=g_date[6])
                ts = tdate.timestamp()
                # g_logger.warning("publish pointcloud ts: %f, %s", ts, str(tdate))
                rclpy_ts = rclpy.time.Time(seconds=int(ts), nanoseconds=(ts-int(ts))*1e9)
                self.header.stamp = rclpy_ts.to_msg()
                msg = point_cloud2.create_cloud(self.header, self.fields, self.np_points)
                self.pub.publish(msg)
                self.cloud_counter += 1
                # g_logger.warning("publish pointcloud[%d] with %d points", self.cloud_counter, msg.width)
            # clear pointcloud
            # self.np_points = np.empty(shape=[0,4], dtype=np.float32)
            self.points_counter = 0
        else:
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

class SocketServerMsop(udpwrapper.SocketServer):
    def __init__(self, host="127.0.0.1", port=2368, len=1206):
        super(SocketServerMsop, self).__init__(host=host, port=port, len=len)
        self.ros_bridge = Ros2BridgeNode()
        self.msop_queue = queue.Queue(maxsize=1000)
        self.CH128X1_MSOP_HEADER = bytearray([0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11])

        self.CH128X1_FRAME_SIZE = 1206  # msop frame contain 171 points, and 9 bytes additional info
        self.CH128X1_SUBFRAME_BYTES = 7
        self.CH128X1_SUBFRAME_POINTS = 171
        self.raw_sub_frames = []
        self.frame_finish = False
        self.discard_counter = 0
        self.decoder_thread = threading.Thread(target=self.decode_msop, args=())
        self.decoder_thread.setDaemon(True)
        self.decoder_thread.start() 

    def post_process(self, msg):
        # the msg passed in is not used
        msop_frame = self.msg
        if not msop_frame:
            return
        try:
            # if not self.msg_queue.full():
            self.msop_queue.put(msop_frame, block=False, timeout=None)
        except queue.Full:
            self.discard_counter += 1
            if self.discard_counter % 1 == 0:
                g_logger.error("msop queue full, discard frame count: %d", self.discard_counter)

    def decode_msop(self):
        g_logger.info("msop decoder thread start.")
        msop_counter = 0
        last_msop_time = time.time()
        while True:
            msop_frame = self.msop_queue.get(block=True, timeout=None)
            for i in range(self.CH128X1_SUBFRAME_POINTS):
                idx0 = i * self.CH128X1_SUBFRAME_BYTES
                idx1 = idx0 + self.CH128X1_SUBFRAME_BYTES
                sub_frame = msop_frame[idx0:idx1]
                # if find header, publish last pointcloud
                self.frame_finish = (sub_frame == self.CH128X1_MSOP_HEADER)
                self.ros_bridge.add_point(sub_frame, self.frame_finish)
                # refresh timestamp with the header frame gps time
                if self.frame_finish:
                    g_date[3] = msop_frame[1197]
                    g_date[4] = msop_frame[1198]
                    g_date[5] = msop_frame[1199]
                    # msop_frame[1200, 1201, 1202, 1203] = uint32(microsec)
                    g_date[6] = struct.unpack(">I", msop_frame[1200:1204])[0]
                    self.frame_finish = False
                    msop_counter += 1
                    g_logger.info("msop decoder finish frame count: %d", msop_counter)
        g_logger.info("msop decoder thread exit.")


class SocketServerDifop(udpwrapper.SocketServer):
    def __init__(self, host="127.0.0.1", port=2368, len=1206):
        super(SocketServerDifop, self).__init__(host=host, port=port, len=len)
        self.CH128X1_DIFOP_HEADER = bytearray([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
        self.CH128X1_DIFOP_TAIL = bytearray([0x0F, 0xF0])
        self.frame_ready = False
        self.difop_counter = 0

    def post_process(self, msg):
        # the msg passed in is not used
        difop_frame = self.msg
        if not difop_frame:
            return

        header = difop_frame[0:8]
        tail = difop_frame[1204:1206]
        # print("msg header: %s", str(header))
        if header == self.CH128X1_DIFOP_HEADER and tail == self.CH128X1_DIFOP_TAIL:
            # g_logger.info("difop frame ready, start decoding")
            self.difop_counter += 1
            self.frame_ready = True
            g_motor_speed = difop_frame[8] << 8
            g_motor_speed = g_motor_speed + difop_frame[9]
            g_logger.info("difop counter: %d, g_motor_speed: %d rpm", self.difop_counter, g_motor_speed)

            g_date[0] = difop_frame[52] + 2000 # (int(ts[0]) - 2000) % 255 # year 0~255
            g_date[1] = difop_frame[53] # int(ts[1]) # month 1~12
            g_date[2] = difop_frame[54] # int(ts[2]) # day 1~31
            g_date[3] = difop_frame[55] # int(ts[3]) # hour 0~23
            g_date[4] = difop_frame[56] # int(ts[4]) # min  0~59
            g_date[5] = difop_frame[57] # int(ts[5]) # sec  0~59
            g_logger.info("difop counter: %d, gps time: %s", self.difop_counter, str(g_date))


if __name__ == "__main__":
    plogging.init_logger(log_dir="./", file_name="lidar_ch128x1_decoder")
    g_logger = plogging.get_logger()
    g_date = [2023, 12, 31, 23, 59, 59, 1001] #[year, mon, day, hr, min, sec, microsec]
    g_motor_speed = 0x0
    rclpy.init(args=None)

    lidar1_msop_receiver = SocketServerMsop(host="127.0.0.1", port=12371, len=1206)
    lidar1_msop_receiver.start_recv_thread(run_background=True)
    lidar1_difop_receiver = SocketServerDifop(host="127.0.0.1", port=12372, len=1206)
    lidar1_difop_receiver.start_recv_thread(run_background=True)

    lidar1_ucwp_sender = udpwrapper.SocketClient(host="127.0.0.1", port=22371)
    counter = 0
    while True:
        msg = bytearray(1206)
        CH128X1_UCWP_HEADER = bytearray([0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA])
        msg[0:8] = CH128X1_UCWP_HEADER
        lidar1_ucwp_sender.send_array(msg)
        time.sleep(5.0)
        counter += 1
        g_logger.info("ucwp client send msg counter: %d", counter)
