import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('carla_manual_control')
    traj_file_path = "{}/data/carla_parking_traj_microlino_0519.json".format(package_share_directory)
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='role_name',
            default_value='ego_vehicle'
        ),
        launch_ros.actions.Node(
            package='carla_manual_control',
            executable='carla_manual_control',
            name=['carla_manual_control_', launch.substitutions.LaunchConfiguration('role_name')],
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'role_name': launch.substitutions.LaunchConfiguration('role_name'),
                    'traj_file': traj_file_path
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
