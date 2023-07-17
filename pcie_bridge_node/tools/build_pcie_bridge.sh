function find_ros2_ws_path() {
    # echo "@i@ --> find dir: ${0}"
    this_script_dir=$( dirname -- "$0"; )
    pwd_dir=$( pwd; )
    ros2_ws_path=${pwd_dir}"/"${this_script_dir}"/../../../"
    echo "${ros2_ws_path}"
}

# ROS2_WS_PATH=/home/hugoliu/github/colcon_ws
ROS2_WS_PATH=$( find_ros2_ws_path )
cd ${ROS2_WS_PATH}

colcon build --packages-up-to pcie_bridge_node