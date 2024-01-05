#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import time, threading, socket, struct, math
from datetime import datetime
import udpwrapper
import plogging

global g_logger

class RadarOculiiEagle(object):
    def __init__(self, idx):
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

        self.frame_handshake = bytearray(self.OCULII_HANDSHAKE_SIZE)
        self.frame_header = bytearray(self.OCULII_HEADER_SIZE)
        self.block_detection = bytearray(self.OCULII_DETECTION_BLOCK_SIZE)
        self.block_track = bytearray(self.OCULII_TRACK_BLOCK_SIZE)
        self.frame_footer = bytearray(self.OCULII_FOOTER_SIZE)
        self.full_frame = bytearray(self.FRAME_MAX_SIZE)

        self.idx = idx
        g_logger.info("radar[%d] FRAME_MAX_SIZE: %d", self.idx, self.FRAME_MAX_SIZE)

        # msop data send from 2369 to 2368
        self.data_udp_client = udpwrapper.SocketClient(host="127.0.0.1", port=19910)

    '''
    # make timestamp from host now() or carla timestamp
    '''
    def make_datetime(self, timestamp=time.time()):
        # year, month, day, hour, min, sec, microsec
        # ['2023', '11', '07', '14', '49', '16', '378058']
        dt = datetime.fromtimestamp(timestamp)
        return dt.strftime("%Y,%m,%d,%H,%M,%S,%f").split(",")

    def make_4bytes_nano_4bytes_sec(self, timestamp=time.time()):
        b4sec = int(timestamp)
        b4nano = (timestamp - b4sec)*1e9
        return (b4nano, b4sec)

    def make_full_frame(self, frame_counter):
        self.full_frame = bytearray(self.FRAME_MAX_SIZE)
        # g_logger.info("full_frame size: %d", len(self.full_frame))
        ### make header
        self.frame_header = bytearray(self.OCULII_HEADER_SIZE)
        self.frame_header[0:8] = self.OCULII_HEADER_MAGIC
        # frame number, uint32
        self.frame_header[8:12] = struct.pack("<I", int(frame_counter))
        # version, float32
        self.frame_header[12:16] = struct.pack("<I", int(self.OCULII_VERSION))
        # num detection uint16
        self.frame_header[16:18] = struct.pack("<H", int(self.OCULII_MAX_POINTS_PER_SCAN))
        # num track uint16
        self.frame_header[18:20] = struct.pack("<H", int(self.OCULII_MAX_TRACK_PER_SCAN))
        # host speed, uint16
        self.frame_header[20:22] = struct.pack("<H", int(60))
        # host angle, uint16
        self.frame_header[22:24] = struct.pack("<H", int(15))
        # reserved 8 bytes
        self.frame_header[24:32] = bytearray(8)
        # range accuracy, uint16
        self.frame_header[32:34] = struct.pack("<H", int(self.HEADER_RANGE_ACCURACY*10000))
        # doppler accuracy, uint16
        self.frame_header[34:36] = struct.pack("<H", int(self.HEADER_DOPPLER_ACCURACY*10000))
        # azimuth accuracy, uint16
        self.frame_header[36:38] = struct.pack("<H", int(self.HEADER_AZIMUTH_ACCURACY*10000))
        # elevation accuracy, uint16
        self.frame_header[38:40] = struct.pack("<H", int(self.HEADER_ELEVATION_ACCURACY*10000))
        # reserved 8 bytes
        self.frame_header[40:48] = bytearray(8)

        ### make detection frame
        dotflag = 0
        power = 7.99 # uint16
        dist = 30.7 # m
        doppler = -18.5 # m/s
        azimuth = 45.2 # deg
        elevation = -7.3 # deg

        dotflag_8b = dotflag
        power_16b = struct.pack("<H", int(power*100))
        dist_16b = struct.pack(">h", int(dist/self.HEADER_RANGE_ACCURACY)-0x200)
        doppler_16b = struct.pack(">h", int(doppler/self.HEADER_DOPPLER_ACCURACY)-0x200)
        azimuth_16b = struct.pack(">h", int(azimuth/self.HEADER_AZIMUTH_ACCURACY)-0x200)
        elevation_16b = struct.pack(">h", int(elevation/self.HEADER_ELEVATION_ACCURACY)-0x200)

        self.block_detection = bytearray(self.OCULII_DETECTION_BLOCK_SIZE)
        self.block_detection[0] = dotflag_8b
        self.block_detection[1:3] = power_16b
        self.block_detection[3] = ((elevation_16b[0]<<6)&0xc0) | ((elevation_16b[1]>>2)&0x3f)
        self.block_detection[4] = ((elevation_16b[1]<<6)&0xc0) | ((azimuth_16b[0]<<4)&0x30) | ((azimuth_16b[1]>>4)&0x0f)
        self.block_detection[5] = ((azimuth_16b[1]<<4)&0xf0) | ((doppler_16b[0]<<2)&0x0c) | ((doppler_16b[1]>>6)&0x03)
        self.block_detection[6] = ((doppler_16b[1]<<2)&0xfc) | (dist_16b[0]&0x03)
        self.block_detection[7] = dist_16b[1]

        ### make track frame
        track_id = 1010
        track_x_pos = 12.0
        track_y_pos = -12.0
        track_z_pos = 12.0
        track_x_dot = -21.0
        track_y_dot = -21.0
        track_z_dot = 21.0
        track_flag = 1
        track_class = 3
        track_conf = 555
        self.block_track = bytearray(self.OCULII_TRACK_BLOCK_SIZE)
        self.block_track[0:4] = struct.pack("<I", int(track_id))
        self.block_track[4:6] = struct.pack("<h", int(track_x_pos*100))
        self.block_track[6:8] = struct.pack("<h", int(track_y_pos*100))
        self.block_track[8:10] = struct.pack("<h", int(track_z_pos*100))
        self.block_track[10:12] = struct.pack("<h", int(track_x_dot*100))
        self.block_track[12:14] = struct.pack("<h", int(track_y_dot*100))
        self.block_track[14:16] = struct.pack("<h", int(track_z_dot*100))
        # reserved 6 bytes
        self.block_track[16:22] = bytearray(6)
        self.block_track[22:24] = struct.pack("<H", int(track_flag))
        self.block_track[24:26] = struct.pack("<H", int(track_class))
        self.block_track[26:28] = struct.pack("<H", int(track_conf))
        self.block_track[28:32] = bytearray(4)

        self.full_frame[0:self.OCULII_HEADER_SIZE] = self.frame_header
        block_start_idx = self.OCULII_HEADER_SIZE
        # g_logger.info("after add header, block_start_idx: %d, frame_header: %d, full_frame size: %d", block_start_idx, len(self.frame_header), len(self.full_frame))

        for i in range(self.OCULII_MAX_POINTS_PER_SCAN):
            self.full_frame[block_start_idx:(block_start_idx+self.OCULII_DETECTION_BLOCK_SIZE)] = self.block_detection
            block_start_idx = block_start_idx + self.OCULII_DETECTION_BLOCK_SIZE
        # g_logger.info("after add detection, block_start_idx: %d, full_frame size: %d, block_detection: %d", block_start_idx, len(self.full_frame), len(self.block_detection))

        for i in range(self.OCULII_MAX_TRACK_PER_SCAN):
            self.full_frame[block_start_idx : block_start_idx+self.OCULII_TRACK_BLOCK_SIZE] = self.block_track
            block_start_idx = block_start_idx+self.OCULII_TRACK_BLOCK_SIZE
        # g_logger.info("after add track, block_start_idx: %d, full_frame size: %d", block_start_idx, len(self.full_frame))

        self.full_frame[block_start_idx : block_start_idx+self.OCULII_FOOTER_SIZE] = self.frame_footer
        g_logger.info("full_frame [%d] size: %d", frame_counter, len(self.full_frame))
        return len(self.full_frame)

    '''
    # make lidar protocol following Leihen-CH128X1, 120*25deg, 1520000pps, 88.7Mbps
    '''
    def start_send_frame(self):
        g_logger.info("radar[%d] msg thread start.", self.idx)
        frame_counter = 0

        while True:
            full_frame_size = self.make_full_frame(frame_counter)

            ### make handshake
            self.frame_handshake[0:8] = self.OCULII_HANDSHAKE_MAGIC
            # frame size, uint32
            self.frame_handshake[8:12] = struct.pack("<I", int(full_frame_size))
            b4nano, b4sec = self.make_4bytes_nano_4bytes_sec(time.time())
            # timestamp nanosec, uint32
            self.frame_handshake[12:16] = struct.pack("<I", int(b4nano))
            # timestamp sec, uint32
            self.frame_handshake[16:20] = struct.pack("<I", int(b4sec))
            # set ptp flag, uint8, sync_out_time = 7
            self.frame_handshake[20] = 7
            # reserved 3 bytes
            self.frame_handshake[21:24] = bytearray(3)

            ### send handshake
            self.data_udp_client.send_array(self.frame_handshake)
            g_logger.info("radar[%d] send handshake [%d], time [%d:%d]", self.idx, frame_counter, b4sec, b4nano)

            start_idx = 0
            subframe_counter = 0
            while (start_idx < full_frame_size):
                subframe_counter += 1
                if (start_idx+self.SUB_FRAME_SIZE) < full_frame_size:
                    self.data_udp_client.send_array(self.full_frame[start_idx:(start_idx+self.SUB_FRAME_SIZE)])
                    start_idx += self.SUB_FRAME_SIZE
                else:
                    self.data_udp_client.send_array(self.full_frame[start_idx:full_frame_size])
                    break
            g_logger.info("radar[%d] frame [%d] subframe_counter [%d]", self.idx, frame_counter, subframe_counter)

            time.sleep(self.OCULII_CYCLE_SEC)
            frame_counter = frame_counter + 1

        g_logger.info("radar[%d] msg thread exit.", self.idx)


if __name__ == "__main__":
    plogging.init_logger(log_dir="./", file_name="radar_oculii_eagle_encoder")
    g_logger = plogging.get_logger()
    radar_0 = RadarOculiiEagle(0)
    radar_0.start_send_frame()

