#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import os, sys, time
import math
import cv2
import numpy as np
#pip install numba scipy numpy-quaternion
import quaternion

# using carla body coordinate, calc homography from svc to bev
# X-forward, Y-right, Z-up,  euler angles in right-hand, pitch: +Y, roll: +X, yaw: -Z.
class CarlaIPM(object):
    def __init__(self, x0_svc=2.58, y0_svc=0.0, z0_svc=0.73, yaw_svc=0.0):
        # from camera to body extrinsic in body-frame, in meter & degree
        self.X0_SVC = x0_svc # front:2.58, left:0.08, rear:-2.62, right:0.08
        self.Y0_SVC = y0_svc # front:0.0, left:-1.1, rear:0.0, right:1.1
        self.Z0_SVC = z0_svc # 0.73
        # these euler angles are from cam to body, 
        # so inverse to extrinsics in carla/helper/scene_wrapper.py
        self.PITCH_SVC = 20.0
        self.ROLL_SVC = 0.0
        self.YAW_SVC = yaw_svc # front:0, left:90, rear:180, right:270

        # bev image params, ps2.0 dataset resolution 600*600
        self.H_BEV_PIXEL = 640
        self.W_BEV_PIXEL = 640
        self.XMAX_BODY_M = 16.0
        # keep w/h ratio, so YMAX_BODY_M could be derived from XMAX_BODY_M
        self.YMAX_BODY_M = self.XMAX_BODY_M * self.W_BEV_PIXEL / self.H_BEV_PIXEL
        self.PIX_DENSITY = self.H_BEV_PIXEL / self.XMAX_BODY_M

        # virtual-bev-camera in body-frame, could be same as svc-camera, 
        # but set to center of ego-vehicle, for IPM synthesis.
        self.X0_BEV = 0.0 # self.X0_SVC + self.XMAX_BODY_M/2
        self.Y0_BEV = 0.0 # self.Y0_SVC
        self.Z0_BEV = self.Z0_SVC
        self.PITCH_BEV = 90.0
        self.ROLL_BEV = 0.0
        self.YAW_BEV = 0.0

        # rotation matrix from camera to body, should never ignored
        # camera coordinate X-right, Y-down, Z-forward
        self.R0_CAM_TO_BODY = np.array([
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        ], dtype=np.float32)
        self.R0_BODY_TO_CAM = np.linalg.inv(self.R0_CAM_TO_BODY)

        self.init_svc_camera_params()
        self.init_bev_camera_params()
        # self.calc_ipm_demo()
        self.calc_ipm()

    def calc_ipm(self):
        # inv_K_bev_cam_43[3, 2]=1 is the key-point for translation
        inv_K_bev_cam_43 = np.concatenate([self.inv_K_bev_cam, np.array([[0, 0, 1]])], axis=0)
        K_svc_cam_34 = np.concatenate([self.K_svc_cam, np.array([[0], [0], [0]])], axis=1)
        # print("inv_K_bev_cam_43: {}, \n K_svc_cam_34: {}".format(inv_K_bev_cam_43, K_svc_cam_34))

        self.H_bev_to_svc = (K_svc_cam_34 @ (self.Ess_svc_body_to_cam @ (self.Ess_bev_cam_to_body @ inv_K_bev_cam_43)))
        self.H_svc_to_bev = np.linalg.inv(self.H_bev_to_svc)
        # print("H_bev_to_svc: {}, \n H_svc_to_bev: {}".format(self.H_bev_to_svc, self.H_svc_to_bev))

    def init_svc_camera_params(self, orientation=0.0):
        self.K_svc_cam = np.array([
            [554.256, 0.0, 960.0],
            [0.0, 554.256, 540.0],
            [0.0,     0.0,   1.0]
        ], dtype=np.float32)
        self.inv_K_svc_cam = np.linalg.inv(self.K_svc_cam)
        # print("K_svc_cam: {}, \n inv_K_svc_cam: {}".format(self.K_svc_cam, self.inv_K_svc_cam))
        # from camera to body, pitch(+Y, right) is 20
        self.R_svc_cam_to_body = self.rot_mat_from_euler(pitch=self.PITCH_SVC, roll=self.ROLL_SVC, yaw=self.YAW_SVC) @ self.R0_CAM_TO_BODY
        self.T_svc_cam_to_body = np.array([[self.X0_SVC], [self.Y0_SVC], [self.Z0_SVC]], dtype=np.float32)
        self.Ess_svc_cam_to_body = np.concatenate((np.concatenate((self.R_svc_cam_to_body, self.T_svc_cam_to_body), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
        self.Ess_svc_body_to_cam = np.linalg.inv(self.Ess_svc_cam_to_body)
        # print("Ess_svc_cam_to_body: {}, \n Ess_svc_body_to_cam: {}".format(self.Ess_svc_cam_to_body, self.Ess_svc_body_to_cam))

    def init_bev_camera_params(self):
        self.f_bev_cam = self.Z0_BEV * self.H_BEV_PIXEL / self.XMAX_BODY_M
        # print("f_bev_cam: {}".format(self.f_bev_cam))
        self.K_bev_cam = np.array([
            [self.f_bev_cam, 0.0, self.W_BEV_PIXEL/2.0],
            [0.0, self.f_bev_cam, self.H_BEV_PIXEL/2.0],
            [0.0,     0.0,                         1.0]
        ], dtype=np.float32)
        # intrinsic normalization, optional, do scale to the homography, but don't affect projection results at all.
        self.K_bev_cam = self.K_bev_cam/self.Z0_BEV
        self.inv_K_bev_cam = np.linalg.inv(self.K_bev_cam)
        # print("K_bev_cam: {}, \n inv_K_bev_cam: {}".format(self.K_bev_cam, self.inv_K_bev_cam))
        # from camera to body, pitch(+Y, right) is 90
        self.R_bev_cam_to_body = self.rot_mat_from_euler(pitch=self.PITCH_BEV, roll=self.ROLL_BEV, yaw=self.YAW_BEV) @ self.R0_CAM_TO_BODY
        # set virtual camera in the center of BEV
        self.T_bev_cam_to_body = np.array([
            [self.X0_BEV],
            [self.Y0_BEV],
            [self.Z0_BEV]
        ], dtype=np.float32)
        self.Ess_bev_cam_to_body = np.concatenate((np.concatenate((self.R_bev_cam_to_body, self.T_bev_cam_to_body), axis=1), np.array([[0, 0, 0, 1]])), axis=0)
        self.Ess_bev_body_to_cam = np.linalg.inv(self.Ess_bev_cam_to_body)
        # print("Ess_bev_cam_to_body: {}, \n Ess_bev_body_to_cam: {}".format(self.Ess_bev_cam_to_body, self.Ess_bev_body_to_cam))

    # pitch, roll, yaw in degree
    # np.quaternion(qw, qx, qy, qz)
    def rot_mat_from_euler(self, pitch=0.0, roll=0.0, yaw=0.0):
        pitch = np.deg2rad(pitch)
        roll = np.deg2rad(roll)
        yaw = np.deg2rad(yaw)
        # roll: +X
        q_roll = np.quaternion(np.cos(roll/2), np.sin(roll/2), 0, 0)
        # pitch: +Y
        q_pitch = np.quaternion(np.cos(pitch/2), 0, np.sin(pitch/2), 0)
        # yaw: -Z
        q_yaw = np.quaternion(np.cos(-yaw/2), 0, 0, np.sin(-yaw/2))
        # the rotation sequence of body_to_cam is ZYX, yaw-roll-pitch
        # so the rotation sequence of cam_to_body is XYZ, roll-pitch-yaw
        q =  q_yaw * q_pitch * q_roll
        rot = quaternion.as_rotation_matrix(q)
        # print("pitch: {:.3f}, roll: {:.3f}, yaw: {:.3f}, \nq: {}, \nrot: {}".format(pitch, roll, yaw, q, rot))
        return rot

    def rot_mat_from_quat(self, q):
        return quaternion.as_rotation_matrix(q)

    # https://answers.opencv.org/question/174548/inverse-perspective-mapping-ipm-on-the-capture-from-camera-feed/
    def calc_ipm_demo(self):
        K_cam = self.K_svc_cam
        T = np.array([
            [0.0],
            [0.0],
            [100.0]
        ], dtype=np.float32)
        R = self.rot_mat_from_euler(pitch=0.0, roll=-70.0, yaw=0.0)
        RT = np.concatenate((np.concatenate((R, T), axis=1), np.array([[0, 0, 0, 1]])), axis=0)

        K_bev = np.array([
            [1.0, 0.0, -960, 0.0],
            [0.0, 1.0, -540, 0.0],
            [0.0, 0.0, 0.0,  0.0],
            [0.0, 0.0, 1.0,  1.0]
        ], dtype=np.float32)
        self.H_bev_to_svc = (K_cam @ (RT @ K_bev))[0:3, 0:3]
        self.H_svc_to_bev = np.linalg.inv(self.H_bev_to_svc)
        print("H_bev_to_svc: {}, \n H_svc_to_bev: {}".format(self.H_bev_to_svc, self.H_svc_to_bev))

    def get_H(self):
        return self.H_svc_to_bev

    def get_cam_center(self):
        cam_center_m = np.array([self.X0_SVC, self.Y0_SVC])
        cam_center_pixel = np.array([self.W_BEV_PIXEL/2 + self.Y0_SVC*self.PIX_DENSITY, self.H_BEV_PIXEL/2-self.X0_SVC*self.PIX_DENSITY])
        print("camera center, meter: {}, pixel: {}".format(cam_center_m, cam_center_pixel))

    def test_H(self):
        image_path = "image/front_camera_view.png"
        img_src = cv2.imread(image_path)
        # img_dst = cv2.warpPerspective(img_src, self.H_svc_to_bev, (img_src.shape[1], img_src.shape[0]))
        H = self.H_svc_to_bev
        # H = np.array([
        #     [ 0.0,  1.0, 0.0],
        #     [ 1.0,  0.0, 0.0],
        #     [ 0.0,  0.0, 1.0]
        # ], dtype=np.float32)
        img_dst = cv2.warpPerspective(img_src, H, (int(self.W_BEV_PIXEL), int(self.H_BEV_PIXEL)))
        cv2.imwrite(image_path + ".ipm.jpg", img_dst)
        cv2.namedWindow("ipm", cv2.WINDOW_NORMAL)
        cv2.imshow("ipm", img_dst)
        cv2.waitKey(-1)

if __name__ == "__main__":
    # svc_front
    ipm = CarlaIPM(x0_svc=1.10, y0_svc=0.0, z0_svc=0.65, yaw_svc=0.0)
    print("svc_front:\n{}".format(ipm.get_H()))
    ipm.get_cam_center()
    # ipm.test_H()
    ipm = CarlaIPM(x0_svc=0.0, y0_svc=-0.74, z0_svc=0.65, yaw_svc=90.0)
    print("svc_left:\n{}".format(ipm.get_H()))
    ipm.get_cam_center()
    # ipm.test_H()
    # sys.exit(0)

    ipm = CarlaIPM(x0_svc=-1.10, y0_svc=0.0, z0_svc=0.65, yaw_svc=180.0)
    print("svc_rear:\n{}".format(ipm.get_H()))
    ipm.get_cam_center()

    ipm = CarlaIPM(x0_svc=0.0, y0_svc=0.74, z0_svc=0.65, yaw_svc=270.0)
    print("svc_right:\n{}".format(ipm.get_H()))
    ipm.get_cam_center()
