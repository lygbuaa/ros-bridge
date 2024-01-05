#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, socket, struct, math
from datetime import datetime
import udpwrapper
import plogging

global g_logger

class LidarCH128X1(object):
    def __init__(self, idx):
        self.CH128X1_FRAME_SIZE = 1206  # msop frame contain 171 points, and 9 bytes additional info
        self.CH128X1_SUBFRAME_BYTES = 7
        self.CH128X1_SUBFRAME_POINTS = 171
        self.CH128X1_MSOP_HEADER = bytearray([0xFF, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x11])
        self.CH128X1_DIFOP_HEADER = bytearray([0xA5, 0xFF, 0x00, 0x5A, 0x11, 0x11, 0x55, 0x55])
        self.CH128X1_DIFOP_TAIL = bytearray([0x0F, 0xF0])
        self.CH128X1_UCWP_HEADER = bytearray([0xAA, 0x00, 0xFF, 0x11, 0x22, 0x22, 0xAA, 0xAA])
        self.CH128X1_UCWP_TAIL = bytearray([0x0F, 0xF0])
        self.msop_frame = bytearray(self.CH128X1_FRAME_SIZE)
        self.sub_frame = bytearray(self.CH128X1_SUBFRAME_BYTES)
        self.lower_fov = -18.0
        self.upper_fov = 7.0
        self.channels = 128
        self.vertical_resolution = (self.upper_fov - self.lower_fov) / self.channels
        self.range = 200.0
        self.idx = idx
        g_logger.info("lidar[%d] channels: %d, vertical_resolution: %.2f", self.idx, self.channels, self.vertical_resolution)

        # msop data send from 2369 to 2368
        self.msop_udp_client = udpwrapper.SocketClient(host="127.0.0.1", port=12371)
        # difop data send from 2368 to 2369 
        self.difop_udp_client = udpwrapper.SocketClient(host="127.0.0.1", port=12372)
        # ucwp data comes into 2368 from 2369, but in case lidar and PC are on the same device, we change ucwp port to 22368
        self.ucwp_udp_server = udpwrapper.SocketServer(host="127.0.0.1", port=22371, len=self.CH128X1_FRAME_SIZE)
        self.ucwp_udp_server.start_recv_thread(run_background=True)

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
    def start_send_frame(self):
        g_logger.info("lidar[%d] msg thread start.", self.idx)
        msop_counter = 0
        difop_counter = 0
        last_msop_time = time.time()
        while True:
            points_count = 153600
            subframe_count = int(points_count / self.CH128X1_SUBFRAME_POINTS) # the max is 8983

            for j in range(subframe_count):
                for i in range(self.CH128X1_SUBFRAME_POINTS):
                    # sub frame per point
                    p = [18.00, -21.07, -1.18, 0.89]
                    # fx = p[0]
                    # fy = p[1]
                    # fz = p[2]
                    # fi = p[3]
                    fx = (msop_counter%100+1.0)*(-1.5) #p[0]
                    fy = (msop_counter%100+1.0)*1.5 #p[1]
                    fz = (msop_counter%100+0.1)*0.05 #p[2]
                    fi = 0.89 #p[3]
                    # distance, float32
                    dist = math.sqrt(fx**2 + fy**2)
                    # horizontal angle, float32
                    hori_angle = math.acos(fy/dist)*57.3
                    # vertical angle, float32
                    vert_angle = math.atan(fz/dist)*57.3
                    # dist = 30.9
                    # hori_angle = 100.0
                    # vert_angle = -14.56
                    # line numer, int32
                    line_num = round((vert_angle - self.lower_fov) / self.vertical_resolution)
                    # g_logger.info("line_num: %d, vert_angle: %.2f", line_num, vert_angle)
                    self.sub_frame[0] = line_num
                    # horizontal angle, resolution 0.01 degree
                    # self.sub_frame[1] = int(hori_angle)
                    # self.sub_frame[2] = int(100*(hori_angle - int(hori_angle)))
                    self.sub_frame[1:3] = struct.pack(">H", int(100*hori_angle))
                    # distance, resolution 1/256 cm
                    dist_cm = int(dist*100)
                    dist_residual = int(256*(dist*100 - dist_cm))
                    self.sub_frame[3:5] = struct.pack(">H", int(dist_cm))
                    self.sub_frame[5] = dist_residual
                    # intensity, 0~255
                    self.sub_frame[6] = int(255*fi)
                    self.msop_frame[i*self.CH128X1_SUBFRAME_BYTES : (i+1)*self.CH128X1_SUBFRAME_BYTES] = self.sub_frame
                    # self.msop_frame[i*self.CH128X1_SUBFRAME_BYTES : (i+1)*self.CH128X1_SUBFRAME_BYTES] = self.CH128X1_MSOP_HEADER
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
                    # g_logger.info("attach msop header")
                self.msop_udp_client.send_array(self.msop_frame)
                # g_logger.info("lidar[%d] send msop frame", self.idx)

            msop_counter += 1
            # g_logger.info("lidar[%d] image [%d] timestamp: %.6f, channels: %d, vert_res: %.2f", self.idx, image.frame, image.timestamp, image.channels, self.vertical_resolution)
            if msop_counter % 10 == 0:
                fps = 10 / (time.time() - last_msop_time)
                last_msop_time = time.time()
                g_logger.info("lidar[%d] msg thread send msop_counter: %d, fps: %.2f", self.idx, msop_counter, fps)

                # send difop frame
                difop_frame = bytearray(self.CH128X1_FRAME_SIZE)
                difop_frame[0:8] = self.CH128X1_DIFOP_HEADER
                motor_speed = 600 #rpm
                difop_frame[8] = (motor_speed & 0xff00) >> 8
                difop_frame[9] = (motor_speed & 0x00ff)
                ts = self.make_timestamp(time.time())
                difop_frame[52] = (int(ts[0]) - 2000) % 255 # year 0~255
                difop_frame[53] = int(ts[1]) # month 1~12
                difop_frame[54] = int(ts[2]) # day 1~31
                difop_frame[55] = int(ts[3]) # hour 0~23
                difop_frame[56] = int(ts[4]) # min  0~59
                difop_frame[57] = int(ts[5]) # sec  0~59
                difop_frame[1204:1206] = self.CH128X1_DIFOP_TAIL

                self.difop_udp_client.send_array(difop_frame)
                difop_counter += 1
                g_logger.info("lidar[%d] send difop frame, counter: %d, motor_speed: %d, timestamp: %s", self.idx, difop_counter, motor_speed, str(ts))
            
            # time.sleep(0.1)

        g_logger.info("lidar[%d] msg thread exit.", self.idx)


if __name__ == "__main__":
    plogging.init_logger(log_dir="./", file_name="lidar_ch128x1_encoder")
    g_logger = plogging.get_logger()
    lidar_0 = LidarCH128X1(0)
    lidar_0.start_send_frame()
