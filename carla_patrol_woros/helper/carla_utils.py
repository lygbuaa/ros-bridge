#!/usr/bin/env python

# Copyright (c) 2021 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" some useful functions in carla """

import glob, os, sys
import argparse
import collections
import datetime
import logging
import math
import random
import re
import weakref

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import VehicleLightState as vls
import argparse
import logging
from numpy import random
from utils import plogging
global g_logger
g_logger = plogging.get_logger()

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)
    if generation.lower() == "all":
        return bps

    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps

    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
            return bps
        else:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print("   Warning! Actor Generation is not valid. No actor will be spawned.")
        return []

def find_weather_presets():
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    name = lambda x: ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name

# find nearest spawn point in map, rotation is ignored
def get_nearest_spawn_point(map, dst_loc, threshold=500.0):
    spawn_points = map.get_spawn_points()
    min_dist = threshold
    nearest_spawn_point = None
    for idx, sp in enumerate(spawn_points):
        # g_logger.info("spawn point [%d], T: %s, R: %s", idx, sp.location, sp.rotation)
        cur_dist = sp.location.distance(dst_loc)
        if cur_dist < min_dist:
            min_dist = cur_dist
            nearest_spawn_point = sp
    if nearest_spawn_point:
        g_logger.info("find nearest spawn point T: %s, dist: %f", nearest_spawn_point.location, min_dist)
    return nearest_spawn_point

if __name__ == '__main__':
    pass
