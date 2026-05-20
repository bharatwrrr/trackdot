# camera_detector_dev.launch.py
#
# Development / bag replay convenience wrapper for camera_detector.
#
# Forces source_mode:=file so aravissrc is never used. The DeepStream pipeline
# reads from an H.264 file via filesrc + h264parse + nvv4l2decoder.
#
# Usage examples:
#
#   # Run both cameras from separate H.264 files
#   ros2 launch camera_detector camera_detector_dev.launch.py \
#     left_uri:=/data/bags/left_cam.h264 \
#     right_uri:=/data/bags/right_cam.h264
#
#   # Run left camera only (right uses YAML default path)
#   ros2 launch camera_detector camera_detector_dev.launch.py \
#     left_uri:=/data/bags/left_cam.h264
#
# NOTE: triton_cam_driver does NOT need to be running in file mode.
#       There is no camera hardware involved.
# ─────────────────────────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('camera_detector')

    # ── Launch arguments ──────────────────────────────────────────────────────

    left_uri_arg = DeclareLaunchArgument(
        'left_uri',
        default_value='',
        description='Path to left camera H.264 file. Overrides YAML [source] path.',
    )

    right_uri_arg = DeclareLaunchArgument(
        'right_uri',
        default_value='',
        description='Path to right camera H.264 file. Overrides YAML [source] path.',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
            [pkg, 'config', 'pgie_config.yaml']
        ),
        description='Path to the main DeepStream pipeline YAML config.',
    )

    # ── cam_left_detector (file mode) ─────────────────────────────────────────

    cam_left_node = Node(
        package='camera_detector',
        executable='camera_detector_node',
        name='cam_left_detector',
        output='screen',
        parameters=[
            {
                # Force file mode — no aravissrc, no camera hardware needed
                'source_mode':  'file',
                'camera_id':    'cam_left',
                'frame_id':     'cam_left_optical',
                'config_file':  LaunchConfiguration('config_file'),
                'source_uri':   LaunchConfiguration('left_uri'),
                'output_topic': 'detections',
                'viz_topic':    'detections_viz',
            },
        ],
        remappings=[
            ('detections',     '/camera/left/detections'),
            ('detections_viz', '/camera/left/detections_viz'),
        ],
    )

    # ── cam_right_detector (file mode) ────────────────────────────────────────

    cam_right_node = Node(
        package='camera_detector',
        executable='camera_detector_node',
        name='cam_right_detector',
        output='screen',
        parameters=[
            {
                'source_mode':  'file',
                'camera_id':    'cam_right',
                'frame_id':     'cam_right_optical',
                'config_file':  LaunchConfiguration('config_file'),
                'source_uri':   LaunchConfiguration('right_uri'),
                'output_topic': 'detections',
                'viz_topic':    'detections_viz',
            },
        ],
        remappings=[
            ('detections',     '/camera/right/detections'),
            ('detections_viz', '/camera/right/detections_viz'),
        ],
    )

    return LaunchDescription([
       left_uri_arg,
        right_uri_arg,
        config_file_arg,
        cam_left_node,
        cam_right_node,
    ]) 
