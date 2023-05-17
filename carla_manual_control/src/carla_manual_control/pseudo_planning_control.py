#!/usr/bin/env python
'''
# read trajectory file, and publish control command.
'''

import datetime
import math
import json
from carla_msgs.msg import CarlaEgoVehicleControl

class PseudoPlanningControl(object):
    def __init__(self, node, filepath="./data/carla_parking_traj.json"):
        self.ros_node = node
        self.traj_d = [] # gear = drive
        self.traj_r = [] # gear = reverse
        self.load_traj(filepath)
        self.idx_d = 0
        self.idx_r = 0
        self.d_true_r_false = True
        self.ctrl_msg = CarlaEgoVehicleControl()

        self.vehicle_control_publisher = self.ros_node.new_publisher(
            CarlaEgoVehicleControl,
            "/carla/ego_vehicle/vehicle_control_cmd",
            qos_profile=10)

    def load_traj(self, filepath):
        lines = []
        try:
            with open(filepath, 'r') as fin:
                lines = fin.readlines()
                self.ros_node.logwarn("load lines: {}".format(len(lines)))
            
            for i in range(len(lines)):
                info = json.loads(lines[i])
                if info["reverse"]:
                    self.traj_r.append(info)
                else:
                    self.traj_d.append(info)
            self.ros_node.logwarn("traj_d: {}, traj_r: {}, total: {}".format(len(self.traj_d), len(self.traj_r), len(lines)))

        except Exception as err:
            self.ros_node.logerr("load traj file error: {}".format(err))

    def next_step(self, x=0.0, y=0.0):
        waypoint = None
        if len(self.traj_d) < 1:
            self.d_true_r_false = False

        if len(self.traj_d) > 0:
            waypoint = self.traj_d.pop(0)
            self.ctrl_msg.gear = 1
        elif len(self.traj_r) > 0:
            waypoint = self.traj_r.pop(0)
            self.ctrl_msg.gear = 0
        else:
            waypoint = None

        if waypoint:
            self.ctrl_msg.reverse = waypoint["reverse"]
            self.ctrl_msg.throttle = waypoint["throttle"]
            self.ctrl_msg.steer = waypoint["steer"]
            self.ctrl_msg.brake = waypoint["brake"]
            self.ctrl_msg.hand_brake = False
        else:
            self.ctrl_msg.throttle = 0.0
            self.ctrl_msg.steer = 0.0
            self.ctrl_msg.brake = 1.0
            self.ctrl_msg.hand_brake = True

        try:
            self.vehicle_control_publisher.publish(self.ctrl_msg)
            self.ros_node.logwarn("send vehicle control, throttle: {}, steer: {}, brake: {}, gear: {}, reverse: {}".format(self.ctrl_msg.throttle, self.ctrl_msg.steer, self.ctrl_msg.brake, self.ctrl_msg.gear, self.ctrl_msg.reverse))
        except Exception as error:
            self.ros_node.logerr("send ctrl msg error: {}".format(error))
