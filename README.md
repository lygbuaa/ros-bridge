# run pseudo_planning_control
0. run carla_manual_control, then press `K_B`, drive the car manually.
1. run carla_manual_control, then press `K_SPACE`, pseudo_planning_control will take over the vehicle.

# save traj file
1. run carla_manual_control, the trajectory will be saved to "/home/hugoliu/github/catkin_ws/src/ros-bridge/data/carla_parking_traj.json"

# run ros-bridge parking demo
0. set up ros2 enviroment `source /opt/ros/foxy/setup.bash`,  `source ./install/setup.bash`
1. start the basic ROS bridge package `ros2 launch carla_ros_bridge carla_ros_bridge_parking.launch.py`
2. spawn the vehicle `ros2 launch carla_spawn_parking carla_spawn_parking.launch.py`
3. manual control `ros2 launch carla_manual_control carla_manual_control.launch.py`
4. start rviz2 `rviz2`
5. record sensor data `ros2 bag record -a`

# build ros-bridge on ros2-foxy
0. install carla python client  `pip install carla==0.9.13`
1. download source codes, follow steps on https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/
2. set up ros2 environment
    `source /opt/ros/foxy/setup.bash`
    `rosdep update`
    `rosdep install --from-paths ros-bridge --ignore-src -r` or `bash ros-bridge/install_dependencies.sh`
3. build packages
    `colcon build`
    if only build carla_manual_control: `colcon build --packages-up-to carla_manual_control`
4. test carla_ros_bridge
    `source ./install/setup.bash`
    `ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py`


# ROS/ROS2 bridge for CARLA simulator

[![Actions Status](https://github.com/carla-simulator/ros-bridge/workflows/CI/badge.svg)](https://github.com/carla-simulator/ros-bridge)
[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)
[![GitHub](https://img.shields.io/github/license/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/blob/master/LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/releases/latest)

 This ROS package is a bridge that enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

![rviz setup](./docs/images/ad_demo.png "AD Demo")

**This version requires CARLA 0.9.13**

## Features

- Provide Sensor Data (Lidar, Semantic lidar, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar, IMU)
- Provide Object Data (Transforms (via [tf](http://wiki.ros.org/tf)), Traffic light status, Visualization markers, Collision, Lane invasion)
- Control AD Agents (Steer/Throttle/Brake)
- Control CARLA (Play/pause simulation, Set simulation parameters)

## Getting started and documentation

Installation instructions and further documentation of the ROS bridge and additional packages are found [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/).
