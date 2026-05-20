from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("gnss_driver"))

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="USB serial port for CANmod.GPS (e.g. /dev/ttyACM0)",
        ),
        Node(
            package="gnss_driver",
            executable="gnss_driver_node",
            name="gnss_driver",
            output="screen",
            parameters=[
                str(pkg_share / "config" / "gnss_params.yaml"),
                {"serial_port": LaunchConfiguration("serial_port")},
            ],
        ),
    ])
