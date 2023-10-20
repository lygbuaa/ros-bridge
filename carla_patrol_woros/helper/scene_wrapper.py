#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" a wrapper for world, vehicle, and sensors """

import glob, os, sys
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref
import pygame
import numpy as np

import carla
from carla import VehicleLightState as vls
from carla import ColorConverter as cc
import argparse
import logging
from numpy import random
from helper.carla_utils import get_actor_blueprints, find_weather_presets, get_actor_display_name, get_nearest_spawn_point
from helper.sensor_wrapper import RadarWrapper, LidarWrapper, CameraWrapper, CollisionSensor, LaneInvasionSensor, GnssSensor, IMUSensor
from utils import plogging
global g_logger
g_logger = plogging.get_logger()

# ==============================================================================
# -- SceneWrapper ---------------------------------------------------------------------
# self.player represent the ego car: self.player = self.world.try_spawn_actor(blueprint, spawn_point)
#

# ==============================================================================

class SceneWrapper(object):
    def __init__(self, carla_world, hud, args, display_manager):
        self.world = carla_world
        self.display_manager = display_manager
        self.parking_lot_location = carla.Location(x=290.0, y=-180.0, z=0.0) # parking lot in Town04
        self.rear_axle_midpoint = carla.Location(x=-1.42, y=0.0, z=0.33)
        self.sync = args.sync
        self.visualization = args.visualization
        self.actor_role_name = args.rolename
        try:
            self.map = self.world.get_map()
            # self.print_spawn_points(self.map)
        except RuntimeError as error:
            g_logger.info('RuntimeError: {}'.format(error))
            g_logger.info('  The server could not send the OpenDRIVE (.xodr) file:')
            g_logger.info('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)

        self.hud = hud
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.imu_sensor = None
        self.radar_sensor = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self._gamma = args.gamma
        self.restart()
        self.world.on_tick(hud.on_world_tick)
        self.recording_enabled = False
        self.recording_start = 0
        self.constant_velocity_enabled = False
        self.show_vehicle_telemetry = False
        self.doors_are_open = False
        self.current_map_layer = 0
        self.map_layer_names = [
            carla.MapLayer.NONE,
            carla.MapLayer.Buildings,
            carla.MapLayer.Decals,
            carla.MapLayer.Foliage,
            carla.MapLayer.Ground,
            carla.MapLayer.ParkedVehicles,
            carla.MapLayer.Particles,
            carla.MapLayer.Props,
            carla.MapLayer.StreetLights,
            carla.MapLayer.Walls,
            carla.MapLayer.All
        ]
        self.godview_camera = None

    def restart(self):
        self.player_max_speed = 1.589  # m/s, this is the max speed of carla.Walker
        self.player_max_speed_fast = 3.713
        # Keep same camera config if the camera manager exists.
        cam_index = 0
        cam_pos_index = 0
        ## Get a random blueprint.# only limited to vehicle.lincoln.mkz_2017
        blueprint = random.choice(get_actor_blueprints(self.world, self._actor_filter, self._actor_generation))
        blueprint.set_attribute('role_name', self.actor_role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])
            g_logger.info("ego vehicle max speed: (%f, %f) m/s", self.player_max_speed, self.player_max_speed_fast)

        # Spawn the player.
        if self.player is not None:
            spawn_point = self.player.get_transform()
            spawn_point.location.z += 2.0
            spawn_point.rotation.roll = 0.0
            spawn_point.rotation.pitch = 0.0
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                g_logger.info('There are no spawn points available in your map/town.')
                g_logger.info('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            # spawn_points = self.map.get_spawn_points()
            # spawn_point = random.choice(spawn_points) if spawn_points else carla.Transform()
            spawn_point = get_nearest_spawn_point(self.map, self.parking_lot_location)
            g_logger.info("player[%s] spawn point T: %s, R: %s", self.actor_role_name, spawn_point.location, spawn_point.rotation)
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.show_vehicle_telemetry = False
            self.modify_vehicle_physics(self.player)

        self.print_vehicle_info(self.player)
        # Set up the sensors.
        self.collision_sensor = CollisionSensor(self.player, self.hud)
        self.lane_invasion_sensor = LaneInvasionSensor(self.player, self.hud)
        self.gnss_sensor = GnssSensor(self.player, location=carla.Location(x=-1.42, y=0.0, z=0.33))
        self.imu_sensor = IMUSensor(self.player)
        actor_type = get_actor_display_name(self.player)
        self.hud.notification(actor_type)

        # godview camera is always on
        if self.visualization == "patrol6":
            self.create_patrol6_set()

        elif self.visualization == "radar": #grid_size = [3, 3]
            self.create_godview_camera(disp_pos=[2, 1])
            self.create_radar_set()

        elif self.visualization == "lidar": #grid_size = [2, 3]
            self.create_godview_camera(disp_pos=[1, 1])
            self.create_lidar_set()

        elif self.visualization == "camera": #grid_size = [4, 3]
            self.create_godview_camera(disp_pos=[3, 2])
            self.create_camera_set()

        elif self.visualization == "svc":
            self.create_godview_camera(disp_pos=[1, 1])
            self.create_svc_set()

        else: #grid_size = [1, 1]
            self.create_godview_camera(disp_pos=[0, 0])

        if self.sync:
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def next_weather(self, reverse=False):
        self._weather_index += -1 if reverse else 1
        self._weather_index %= len(self._weather_presets)
        preset = self._weather_presets[self._weather_index]
        self.hud.notification('Weather: %s' % preset[1])
        self.player.get_world().set_weather(preset[0])

    def next_map_layer(self, reverse=False):
        self.current_map_layer += -1 if reverse else 1
        self.current_map_layer %= len(self.map_layer_names)
        selected = self.map_layer_names[self.current_map_layer]
        self.hud.notification('LayerMap selected: %s' % selected)

    def load_map_layer(self, unload=False):
        selected = self.map_layer_names[self.current_map_layer]
        if unload:
            self.hud.notification('Unloading map layer: %s' % selected)
            self.world.unload_map_layer(selected)
        else:
            self.hud.notification('Loading map layer: %s' % selected)
            self.world.load_map_layer(selected)

    def toggle_radar(self):
        pass
        # if self.radar_sensor is None:
        #     self.radar_sensor = RadarSensor(self.player)
        # elif self.radar_sensor.sensor is not None:
        #     self.radar_sensor.sensor.destroy()
        #     self.radar_sensor = None

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def tick(self, clock):
        self.hud.tick(self, clock)
        # self.print_vehicle_status(self.player)

    def render(self, display):
        # self.godview_camera.render()
        # self.hud.render(display)
        pass

    def destroy_sensors(self):
        pass
        # self.camera_manager.sensor.destroy()
        # self.camera_manager.sensor = None
        # self.camera_manager.index = None

    def destroy(self):
        # if self.radar_sensor is not None:
        #     self.toggle_radar()
        sensors = [
            # self.camera_manager.sensor,
            self.collision_sensor.sensor,
            self.lane_invasion_sensor.sensor,
            self.gnss_sensor.sensor,
            self.imu_sensor.sensor]
        for sensor in sensors:
            if sensor is not None:
                sensor.stop()
                sensor.destroy()
        if self.player is not None:
            self.player.destroy()

    def print_spawn_points(self, map):
        spawn_points = map.get_spawn_points()
        for idx, sp in enumerate(spawn_points):
            g_logger.info("spawn point [%d], T: %s, R: %s", idx, sp.location, sp.rotation)
        # g_logger.info("spawn points: {}".format(map.get_spawn_points()))

    def print_vehicle_info(self, ego_vehicle):
        # print ego vehicle info
        ego_bbx = ego_vehicle.bounding_box
        g_logger.info("ego bounding box, extent: %s, location: %s, rotation: %s", ego_bbx.extent, ego_bbx.location, ego_bbx.rotation)
        ego_spd_limit = ego_vehicle.get_speed_limit()
        ego_steer_fl = ego_vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
        g_logger.info("ego speed limit: %f m/s, ego steer front left wheel: %f", ego_spd_limit, ego_steer_fl)

    def print_vehicle_status(self, ego_vehicle):
        ego_control = ego_vehicle.get_control()
        g_logger.info("ego steer: %f, throttle: %f, brake: %f", ego_control.steer, ego_control.throttle, ego_control.brake)

    ###########################################
    ## make sensors
    ###########################################
    def create_godview_camera(self, disp_pos=[0, 0]):
        self.godview_camera = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.0, y=0.0, z=20.0), carla.Rotation(pitch=-90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0'},
            display_pos=disp_pos
        )

    def create_radar_set(self):
        self.radar_lrr = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=2.53, y=0.0, z=0.63), carla.Rotation()),
            attached=self.player,
            sensor_options={'horizontal_fov':'9.0', 'points_per_second':'15000', 'range':'150.0', 'sensor_tick':'0.05', 'vertical_fov':'10.0'},
            display_pos=[0, 1]
        )
        self.radar_srr_front_left = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=2.23, y=-1.0, z=0.63), carla.Rotation(yaw=-45.0)),
            attached=self.player,
            sensor_options={'horizontal_fov':'150.0', 'points_per_second':'15000', 'range':'100.0', 'sensor_tick':'0.05', 'vertical_fov':'10.0'},
            display_pos=[0, 0]
        )
        self.radar_srr_front_right = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=2.23, y=1.0, z=0.63), carla.Rotation(yaw=45.0)),
            attached=self.player,
            sensor_options={'horizontal_fov':'150.0', 'points_per_second':'15000', 'range':'100.0', 'sensor_tick':'0.05', 'vertical_fov':'10.0'},
            display_pos=[0, 2]
        )
        self.radar_srr_rear_left = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=-2.42, y=-1.0, z=0.63), carla.Rotation(yaw=-135.0)),
            attached=self.player,
            sensor_options={'horizontal_fov':'150.0', 'points_per_second':'15000', 'range':'100.0', 'sensor_tick':'0.05', 'vertical_fov':'10.0'},
            display_pos=[1, 0]
        )
        self.radar_srr_rear_right = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=-2.42, y=-1.0, z=0.63), carla.Rotation(yaw=135.0)),
            attached=self.player,
            sensor_options={'horizontal_fov':'150.0', 'points_per_second':'15000', 'range':'100.0', 'sensor_tick':'0.05', 'vertical_fov':'10.0'},
            display_pos=[1, 2]
        )

    def create_lidar_set(self):
        self.lidar_front = LidarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='LiDAR',
            transform=carla.Transform(carla.Location(x=2.48, y=0.0, z=0.73), carla.Rotation()),
            attached=self.player,
            sensor_options={'channels':'64', 'range':'100.0', 'points_per_second':'1536000', 'rotation_frequency':'10', 'horizontal_fov':'120.0', 'upper_fov':'15.0', 'lower_fov':'-10.0', 'sensor_tick':'0.1'},
            display_pos=[0, 1]
        )
        self.lidar_left = LidarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='LiDAR',
            transform=carla.Transform(carla.Location(x=2.18, y=-1.0, z=0.73), carla.Rotation(yaw=-90.0)),
            attached=self.player,
            sensor_options={'channels':'64', 'range':'100.0', 'points_per_second':'1536000', 'rotation_frequency':'10', 'horizontal_fov':'120.0', 'upper_fov':'15.0', 'lower_fov':'-10.0', 'sensor_tick':'0.1'},
            display_pos=[0, 0]
        )
        self.lidar_right = LidarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='LiDAR',
            transform=carla.Transform(carla.Location(x=2.18, y=1.0, z=0.73), carla.Rotation(yaw=90.0)),
            attached=self.player,
            sensor_options={'channels':'64', 'range':'100.0', 'points_per_second':'1536000', 'rotation_frequency':'10', 'horizontal_fov':'120.0', 'upper_fov':'15.0', 'lower_fov':'-10.0', 'sensor_tick':'0.1'},
            display_pos=[0, 2]
        )

    def create_camera_set(self):
        self.camera_flwc = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.48, y=0.0, z=1.53), carla.Rotation()),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[0, 1]
        )
        self.camera_fltc = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.48, y=0.0, z=1.58), carla.Rotation()),
            attached=self.player,
            sensor_options={'fov':'30.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[1, 1]
        )
        self.camera_sflc_left = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.63, y=-1.0, z=1.03), carla.Rotation(yaw=-50.0)),
            attached=self.player,
            sensor_options={'fov':'100.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[0, 0]
        )
        self.camera_sflc_right = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.63, y=1.0, z=1.03), carla.Rotation(yaw=50.0)),
            attached=self.player,
            sensor_options={'fov':'100.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[0, 2]
        )
        self.camera_srlc_left = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=-0.42, y=-1.0, z=1.03), carla.Rotation(yaw=-130.0)),
            attached=self.player,
            sensor_options={'fov':'100.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[1, 0]
        )
        self.camera_srlc_right = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=-0.42, y=1.0, z=1.03), carla.Rotation(yaw=130.0)),
            attached=self.player,
            sensor_options={'fov':'100.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[1, 2]
        )
        self.camera_rlc = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=-1.22, y=0.0, z=1.53), carla.Rotation(yaw=180.0)),
            attached=self.player,
            sensor_options={'fov':'60.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[3, 0]
        )
        self.camera_svc_front = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=2.58, y=0.0, z=0.73), carla.Rotation(pitch=-30.0, yaw=0.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033', 'lens_k':'0.0', 'lens_x_size':'0.0', 'lens_y_size':'0.0'},
            display_pos=[2, 1]
        )
        self.camera_svc_rear = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=-2.62, y=0.0, z=0.73), carla.Rotation(pitch=-30.0, yaw=180.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[3, 1]
        )
        self.camera_svc_left = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.08, y=-1.1, z=0.73), carla.Rotation(pitch=-30.0, yaw=-90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[2, 0]
        )
        self.camera_svc_right = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.08, y=1.1, z=0.73), carla.Rotation(pitch=-30.0, yaw=90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[2, 2]
        )

    def create_svc_set(self):
        self.camera_svc_front = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=2.58, y=0.0, z=0.73), carla.Rotation(pitch=-20.0, yaw=0.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[0, 1]
        )
        self.camera_svc_rear = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=-2.62, y=0.0, z=0.73), carla.Rotation(pitch=-20.0, yaw=180.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[2, 1]
        )
        self.camera_svc_left = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.08, y=-1.1, z=0.73), carla.Rotation(pitch=-20.0, yaw=-90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[1, 0]
        )
        self.camera_svc_right = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=0.08, y=1.1, z=0.73), carla.Rotation(pitch=-20.0, yaw=90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[1, 2]
        )

        self.semantic_svc_front = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='SemanticCamera',
            transform=carla.Transform(carla.Location(x=2.58, y=0.0, z=0.73), carla.Rotation(pitch=-20.0, yaw=0.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[0, 0]
        )
        self.semantic_svc_rear = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='SemanticCamera',
            transform=carla.Transform(carla.Location(x=-2.62, y=0.0, z=0.73), carla.Rotation(pitch=-20.0, yaw=180.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[2, 2]
        )
        self.semantic_svc_left = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='SemanticCamera',
            transform=carla.Transform(carla.Location(x=0.08, y=-1.1, z=0.73), carla.Rotation(pitch=-20.0, yaw=-90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[2, 0]
        )
        self.semantic_svc_right = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='SemanticCamera',
            transform=carla.Transform(carla.Location(x=0.08, y=1.1, z=0.73), carla.Rotation(pitch=-20.0, yaw=90.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.033'},
            display_pos=[0, 2]
        )

    def create_patrol6_set(self):
        self.camera_flwc = CameraWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=1.5, y=0.0, z=1.5), carla.Rotation(yaw=0.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.05'},
            display_pos=[0, 1]
        )
        self.camera_rlc = CameraWrapper(
            world=self.world,
            display_man=self.display_manager, 
            sensor_type='RGBCamera',
            transform=carla.Transform(carla.Location(x=-1.5, y=0.0, z=1.5), carla.Rotation(yaw=180.0)),
            attached=self.player,
            sensor_options={'fov':'120.0', 'image_size_x':'1920', 'image_size_y':'1080', 'sensor_tick':'0.05'},
            display_pos=[1, 1]
        )

        self.lidar_front = LidarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='LiDAR',
            transform=carla.Transform(carla.Location(x=2.0, y=0.0, z=1.5), carla.Rotation(yaw=0.0)),
            attached=self.player,
            sensor_options={'channels':'128', 'range':'100.0', 'points_per_second':'1536000', 'rotation_frequency':'10', 'horizontal_fov':'120.0', 'upper_fov':'15.0', 'lower_fov':'-25.0', 'sensor_tick':'0.1'},
            display_pos=[0, 0]
        )
        self.lidar_rear = LidarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='LiDAR',
            transform=carla.Transform(carla.Location(x=-2.0, y=0.0, z=1.5), carla.Rotation(yaw=180.0)),
            attached=self.player,
            sensor_options={'channels':'128', 'range':'100.0', 'points_per_second':'1536000', 'rotation_frequency':'10', 'horizontal_fov':'120.0', 'upper_fov':'15.0', 'lower_fov':'-25.0', 'sensor_tick':'0.1'},
            display_pos=[1, 0]
        )

        self.radar_front = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=2.0, y=0.0, z=1.5), carla.Rotation(yaw=0.0)),
            attached=self.player,
            sensor_options={'horizontal_fov':'30.0', 'points_per_second':'1500', 'range':'150.0', 'vertical_fov':'15.0'},
            display_pos=[0, 2]
        )
        self.radar_rear = RadarWrapper(
            world=self.world, 
            display_man=self.display_manager, 
            sensor_type='Radar',
            transform=carla.Transform(carla.Location(x=-2.0, y=0.0, z=1.5), carla.Rotation(yaw=180.0)),
            attached=self.player,
            sensor_options={'horizontal_fov':'30.0', 'points_per_second':'1500', 'range':'150.0', 'vertical_fov':'15.0'},
            display_pos=[1, 2]
        )