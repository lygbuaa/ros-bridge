#!/bin/bash

function find_ros2_ws_path() {
    # echo "@i@ --> find dir: ${0}"
    this_script_dir=$( dirname -- "$0"; )
    pwd_dir=$( pwd; )
    ros2_ws_path=${pwd_dir}"/"${this_script_dir}"/../../"
    echo "${ros2_ws_path}"
}

# ROS2_WS_PATH=/home/hugoliu/github/colcon_ws
ROS2_WS_PATH=$( find_ros2_ws_path )
cd ${ROS2_WS_PATH}

colcon build --packages-up-to carla_ros_bridge
colcon build --packages-up-to carla_spawn_parking
colcon build --packages-up-to carla_manual_control
colcon build --packages-up-to can_bridge_node