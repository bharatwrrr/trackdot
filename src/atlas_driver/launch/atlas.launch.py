from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        AppendEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value='/opt/aeva/atlas-api/ros2/library'
        ),

        Node(
            package='atlas_driver',
            executable='ros2_aeva_publisher',
            name='atlas_driver',
            output='screen',
            arguments=['10.42.0.45', 'atlas'],
        )
    ])
