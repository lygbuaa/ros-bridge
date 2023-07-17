import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('pcie_bridge_node')
    # package_rootdir = "{}/../../../../se-autodriving-apa/".format(package_share_directory)
    config_file_path = "{}/config/aumo_s2.yaml".format(package_share_directory)
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pcie_bridge_node',
            executable='pcie_bridge_node_bin',
            name='pcie_bridge_node_name',
            # log-level: DEBUG/INFO/WARN/ERROR/FATAL, do `colcon build` before launch.
            arguments=[config_file_path, '--ros-args', '--log-level', 'INFO']
        ),
  ])