#!/bin/bash

function find_ros2_ws_path() {
    # echo "@i@ --> find dir: ${0}"
    this_script_dir=$( dirname -- "$0"; )
    pwd_dir=$( pwd; )
    ros2_ws_path=${pwd_dir}"/"${this_script_dir}"/../../"
    echo "${ros2_ws_path}"
}

export ROS_HOME=~/.ros
# set ros2-log-format, goto https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity} {time}] [{file_name}:{line_number}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1

# setup env
# export PRJ_PATH=/home/hugoliu/github/catkin_ws/
export PRJ_PATH=$( find_ros2_ws_path )
# set ros2-log-dir
export ROS_LOG_DIR=${PRJ_PATH}/log
source /opt/ros/foxy/setup.bash
source ${PRJ_PATH}/install/setup.bash

ros2 launch carla_ros_bridge carla_ros_bridge_parking.launch.py &
ros2 launch carla_spawn_parking carla_spawn_parking.launch.py &
ros2 launch carla_manual_control carla_manual_control.launch.py

# kill carla_ros_bridge_parking
kill -15 `ps -e | grep "bridge" | awk '{print $1}'`
# kill carla_spawn_parking
kill -15 `ps -e | grep "carla_spawn" | awk '{print $1}'`
