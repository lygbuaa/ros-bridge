#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

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

import math
import random
sys.path.append("../utils")
import plogging
global g_logger
plogging.init_logger(log_dir="./", file_name="carla_client")
g_logger = plogging.get_logger()


def get_transform(vehicle_location, angle, d=6.4):
    a = math.radians(angle)
    location = carla.Location(d * math.cos(a), d * math.sin(a), 2.0) + vehicle_location
    return carla.Transform(location, carla.Rotation(yaw=180 + angle, pitch=-15))

def print_vehicle_info(ego_vehicle):
    # print ego vehicle info
    ego_bbx = ego_vehicle.bounding_box
    g_logger.info("%s bounding box, extent: %s, location: %s, rotation: %s", ego_vehicle.type_id, ego_bbx.extent, ego_bbx.location, ego_bbx.rotation)
    ego_spd_limit = ego_vehicle.get_speed_limit()
    ego_steer_fl = ego_vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
    g_logger.info("%s speed limit: %s m/s, ego steer front left wheel: %s", ego_vehicle.type_id, str(ego_spd_limit), str(ego_steer_fl))

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    spectator = world.get_spectator()
    vehicle_blueprints = world.get_blueprint_library().filter('vehicle')

    location = random.choice(world.get_map().get_spawn_points()).location

    for blueprint in vehicle_blueprints:
        transform = carla.Transform(location, carla.Rotation(yaw=-45.0))
        vehicle = world.spawn_actor(blueprint, transform)

        try:
            print(vehicle.type_id)
            print_vehicle_info(vehicle)

            angle = 0
            while angle < 60:
                timestamp = world.wait_for_tick().timestamp
                angle += timestamp.delta_seconds * 60.0
                spectator.set_transform(get_transform(vehicle.get_location(), angle - 90))

        finally:

            vehicle.destroy()


if __name__ == '__main__':

    main()
