# camera_detector.launch.py
#
# Launches two instances of camera_detector_node: cam_left_detector and
# cam_right_detector. Each instance owns one physical camera end-to-end:
# capture (aravissrc) → inference (DeepStream) → detections (ROS topic).
#
# Source mode is controlled by pipeline_source.yml (default: native).
# Override at launch time with:
#   ros2 launch camera_detector camera_detector.launch.py source_mode:=file
#
# For bag replay convenience, use camera_detector_dev.launch.py instead.
#
# ── Launch ordering note ──────────────────────────────────────────────────────
# This file is NOT called directly in production. It is included by
# bringup/launch/full_system.launch.py which wraps it in a TimerAction (3 s)
# to guarantee triton_cam_driver has configured + released the cameras before
# aravissrc tries to connect.
# ─────────────────────────────────────────────────────────────────────────────

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg = FindPackageShare('camera_detector')

    # ── Launch arguments ──────────────────────────────────────────────────────

    source_mode_arg = DeclareLaunchArgument(
        'source_mode',
        default_value='native',
        description=(
            "Pipeline source mode. "
            "'native' = aravissrc (production, no ROS on pixel path). "
            "'file'   = filesrc + h264parse (dev / bag replay)."
        ),
    )

    # Main DeepStream YAML: contains [streammux], [primary-gie], [secondary-gie1],
    # [tracker], and [source] sections. [source] is only read in file mode.
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution(
            [pkg, 'config', 'detector_config.yml']
        ),
        description='Path to the main DeepStream pipeline YAML config.',
    )

    # source_uri is only meaningful in file mode. Ignored in native mode.
    source_uri_arg = DeclareLaunchArgument(
        'source_uri',
        default_value='',
        description='(file mode only) Path to H.264 source file. Overrides YAML [source] path.',
    )

    # ── Parameter files ───────────────────────────────────────────────────────

    pipeline_source_yaml = PathJoinSubstitution(
        [pkg, 'config', 'pipeline_source.yml']
    )

    # ── cam_left_detector ─────────────────────────────────────────────────────

    cam_left_node = Node(
        package='camera_detector',
        executable='camera_detector_node',
        name='cam_left_detector',
        output='screen',
        parameters=[
            # 1. Load per-instance source config (serial, dims, viz params)
            pipeline_source_yaml,
            # 2. Runtime overrides — these win over pipeline_source.yaml values
            {
                'camera_id':    'cam_left',
                'frame_id':     'cam_left_optical',
                'config_file':  LaunchConfiguration('config_file'),
                'source_mode':  LaunchConfiguration('source_mode'),
                'source_uri':   LaunchConfiguration('source_uri'),
                'output_topic': 'detections',
            },
        ],
        remappings=[
            # Namespace outputs under /camera/lef/ so lidar_cam_fusion
            # can subscribe with clean topic names
            ('detections',     '/camera/left/detections'),
            ('detections_viz', '/camera/left/detections_viz'),
            ('debug_image',    '/camera/left/images/compressed'),
        ],
    )

    # ── cam_right_detector ────────────────────────────────────────────────────

    cam_right_node = Node(
        package='camera_detector',
        executable='camera_detector_node',
        name='cam_right_detector',
        output='screen',
        parameters=[
            pipeline_source_yaml,
            {
                'camera_id':    'cam_right',
                'frame_id':     'cam_right_optical',
                'config_file':  LaunchConfiguration('config_file'),
                'source_mode':  LaunchConfiguration('source_mode'),
                'source_uri':   LaunchConfiguration('source_uri'),
                'output_topic': 'detections',
            },
        ],
        remappings=[
            ('detections',     '/camera/right/detections'),
            ('detections_viz', '/camera/right/detections_viz'),
            ('debug_image',    '/camera/right/images/compressed'),
        ],
    )

    return LaunchDescription([
        source_mode_arg,
        config_file_arg,
        source_uri_arg,
        cam_left_node,
        cam_right_node,
    ])
