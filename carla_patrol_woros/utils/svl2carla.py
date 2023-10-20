#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time
import math
import numpy as np
#pip install numba scipy numpy-quaternion
import quaternion

# center point: midpoint of rear axle
class LGSVL_Transform(object):
    x = 0.0
    y = 0.0
    z = 0.0
    pitch = 0.0
    yaw = 0.0
    roll = 0.0

    def __init__(self, x=0.0, y=0.0, z=0.0, pitch=0.0, roll=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.z = z
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw

    def show(self):
        print("lgsvl transform:\n trans.xyz: {:.4f}, {:.4f}, {:.4f}\n euler.pyr: {:.4f}, {:.4f}, {:.4f}\n".format(self.x, self.y, self.z, self.pitch, self.yaw, self.roll))

# center point: midpoint of rear axle
class Apollo_Transform(object):
    x = 0.0
    y = 0.0
    z = 0.0
    pitch = 0.0
    yaw = 0.0
    roll = 0.0
    qx = 0.0
    qy = 0.0
    qz = 0.0
    qw = 0.0

    def show(self):
        print("apollo transform:\n trans.xyz: {:.4f}, {:.4f}, {:.4f}\n euler.pyr: {:.4f}, {:.4f}, {:.4f}\n q.xyzw: {:.4f}, {:.4f}, {:.4f}, {:.4f}\n".format(self.x, self.y, self.z, self.pitch, self.yaw, self.roll, self.qx, self.qy, self.qz, self.qw))

# center of vehicle, on the ground
class Carla_Transform(object):
    x = 0.0
    y = 0.0
    z = 0.0
    pitch = 0.0
    yaw = 0.0
    roll = 0.0

    def __init__(self):
        #  vehicle.lincoln.mkz_2017, midpoint of rear axle in carla coordinate 
        #  vehicle.lincoln.mkz_2017，3d-bbox (x=2.450842, y=1.064162, z=0.755373), wheel_base=2.84,  wheel_rolling_radius=0.33
        self.center_x = -1.42
        self.center_y = 0.0
        self.center_z = 0.33

    def show(self):
        print("carla transform:\n x: {:.4f},\n y: {:.4f},\n z:{:.4f}\n pitch: {:.4f},\n yaw: {:.4f},\n roll:{:.4f}\n".format(self.x, self.y, self.z, self.pitch, self.yaw, self.roll))

def LGSVL_2_Carla(trans_lgsvl):
    trans_carla = Carla_Transform()
    trans_carla.x = trans_lgsvl.z + trans_carla.center_x
    trans_carla.y = trans_lgsvl.x + trans_carla.center_y
    trans_carla.z = trans_lgsvl.y + trans_carla.center_z

    trans_carla.pitch = -1*trans_lgsvl.pitch
    trans_carla.roll = -1*trans_lgsvl.roll
    trans_carla.yaw = trans_lgsvl.yaw

    return trans_carla

def LGSVL_2_Apollo(trans_lgsvl):
    trans_apollo = Apollo_Transform()
    trans_apollo.x = trans_lgsvl.x
    trans_apollo.y = trans_lgsvl.z
    trans_apollo.z = trans_lgsvl.y

    trans_apollo.pitch = trans_lgsvl.pitch * -1.0
    trans_apollo.yaw = trans_lgsvl.yaw * -1.0
    trans_apollo.roll = trans_lgsvl.roll * -1.0

    angle = trans_apollo.pitch / 57.3
    q_pitch = np.quaternion(np.cos(angle/2), np.sin(angle/2), 0, 0)
    angle = trans_apollo.roll / 57.3
    q_roll = np.quaternion(np.cos(angle/2), 0, np.sin(angle/2), 0)
    angle = trans_apollo.yaw / 57.3
    q_yaw = np.quaternion(np.cos(angle/2), 0, 0, np.sin(angle/2))
    q = q_pitch * q_roll * q_yaw

    # sensor follow the velodyne128 frame
    # q_sensor = np.quaternion(0.7071, 0.0, 0.0, 0.7071)
    q_sensor = np.quaternion(0.7071, -0.7071, 0.0, 0.0)
    q = q*q_sensor

    trans_apollo.qx = q.x
    trans_apollo.qy = q.y
    trans_apollo.qz = q.z
    trans_apollo.qw = q.w

    return trans_apollo

def quat_2_rot(qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    q = np.quaternion(qw, qx, qy, qz)
    rot = quaternion.as_rotation_matrix(q)
    print("q: {}, rot: {}".format(q, rot))
    return rot

# lgsvl using left-hand, +x: right (pitch), +y: up (yaw), +z: forward (roll)
# input: transform:{x, y, z, pitch, yaw, roll}
# apollo using right-hand, +x: right (pitch), +y: forward (roll), +z: up (yaw)
# output: translation: {x, y, z}  rotation: {x, y, z, w}
if __name__ == "__main__":
    svl_cam_flwc = LGSVL_Transform(x=0.0, y=1.2, z=1.9, pitch=0.0, roll=0.0, yaw=0.0)
    svl_cam_fltc = LGSVL_Transform(x=0.0, y=1.25, z=1.9, pitch=0.0, roll=0.0, yaw=0.0)

    svl_cam_sflc_left = LGSVL_Transform(x=-1.0, y=0.7, z=2.0, pitch=0.0, roll=0.0, yaw=-50.0)
    svl_cam_sflc_right = LGSVL_Transform(x=1.0, y=0.7, z=2.0, pitch=0.0, roll=0.0, yaw=50.0)
    svl_cam_srlc_left = LGSVL_Transform(x=-1.0, y=0.7, z=1.0, pitch=0.0, roll=0.0, yaw=-130.0)
    svl_cam_srlc_right = LGSVL_Transform(x=1.0, y=0.7, z=1.0, pitch=0.0, roll=0.0, yaw=130.0)
    svl_cam_rlc = LGSVL_Transform(x=0.0, y=1.2, z=0.2, pitch=0.0, roll=0.0, yaw=180.0)

    svl_cam_svc_front = LGSVL_Transform(x=0.0, y=0.4, z=4.0, pitch=30.0, roll=0.0, yaw=0.0)
    svl_cam_svc_rear = LGSVL_Transform(x=0.0, y=0.4, z=-1.2, pitch=30.0, roll=0.0, yaw=180.0)
    svl_cam_svc_left = LGSVL_Transform(x=-1.1, y=0.4, z=1.5, pitch=30.0, roll=0.0, yaw=-90.0)
    svl_cam_svc_right = LGSVL_Transform(x=1.1, y=0.4, z=1.5, pitch=30.0, roll=0.0, yaw=90.0)

    svl_lidar_front = LGSVL_Transform(x=0.0, y=0.4, z=3.9, pitch=0.0, roll=0.0, yaw=0.0)
    svl_lidar_left = LGSVL_Transform(x=-1.0, y=0.4, z=3.6, pitch=0.0, roll=0.0, yaw=-90.0)
    svl_lidar_right = LGSVL_Transform(x=1.0, y=0.4, z=3.6, pitch=0.0, roll=0.0, yaw=90.0)

    svl_radar_lrr = LGSVL_Transform(x=0.0, y=0.3, z=3.95, pitch=0.0, roll=0.0, yaw=0.0)
    svl_radar_srr_front_left = LGSVL_Transform(x=-1.0, y=0.3, z=3.65, pitch=0.0, roll=0.0, yaw=-45.0)
    svl_radar_srr_front_right = LGSVL_Transform(x=1.0, y=0.3, z=3.65, pitch=0.0, roll=0.0, yaw=45.0)
    svl_radar_srr_rear_left = LGSVL_Transform(x=-1.0, y=0.3, z=-1.0, pitch=0.0, roll=0.0, yaw=-135.0)
    svl_radar_srr_rear_right = LGSVL_Transform(x=1.0, y=0.3, z=-1.0, pitch=0.0, roll=0.0, yaw=135.0)

    # LGSVL_2_Carla(svl_radar_srr_rear_right).show()

    quat_2_rot(qx=-0.57922796534, qy=0.57922796534, qz=-0.405579787673, qw=0.405579787673)
