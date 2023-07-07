import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('can_bridge_node')
    # package_rootdir = "{}/../../../../se-autodriving-apa/".format(package_share_directory)
    config_file_path = "{}/where_is_the_config.yaml".format(package_share_directory)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='can_bridge_node',
            executable='can_bridge_node_bin',
            name='can_bridge_node_name',
            # log-level: DEBUG/INFO/WARN/ERROR/FATAL, do `colcon build` before launch.
            arguments=[config_file_path, '--ros-args', '--log-level', 'INFO']
        ),
  ])