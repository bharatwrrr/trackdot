from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('lidar_cam_fusion')
    config_file = os.path.join(pkg_share, 'overlay_config.yaml')

    return LaunchDescription([
        Node(
            package='lidar_cam_fusion',
            executable='lidar_camera_overlay_node',
            name='lidar_camera_overlay_node',
            output='screen',
            parameters=[config_file],
            emulate_tty=True,
        )
    ])
