import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def file_exists(package_name, relative_path):
    """Check if a file exists in a package."""
    try:
        package_dir = get_package_share_directory(package_name)
        full_path = os.path.join(package_dir, relative_path)
        return os.path.exists(full_path)
    except Exception:
        return False


def generate_launch_description():
    # Launch arguments
    include_h3 = LaunchConfiguration('include_h3', default='false')
    data_collection_mode = LaunchConfiguration('data_collection_mode', default='false')
    
    declare_include_h3 = DeclareLaunchArgument(
        'include_h3',
        default_value='false',
        description='Include H3 sensor driver'
    )
    
    declare_data_collection_mode = DeclareLaunchArgument(
        'data_collection_mode',
        default_value='false',
        description='Camera driver publishes frames directly (true for bag recording). '
                    'If false, camera driver releases device for camera_detector.'
    )
    
    # List of launch files to include: (package, path, description, required, pass_args)
    # pass_args is a dict of launch arguments to pass to this specific launch
    launch_configs = [
        ('triton_cam_driver', 'launch/triton_cam.launch.py', 'Triton cameras', True, 
         {'data_collection_mode': data_collection_mode}),
        ('atlas_driver', 'launch/atlas.launch.py', 'Aeva Atlas LiDAR', True, {}),
        ('gnss_driver', 'launch/gnss.launch.py', 'GNSS receiver', True, {}),
        ('can_driver', 'launch/can.launch.py', 'CAN bus', True, {}),
        ('ego_state', 'launch/ego_state.launch.py', 'Ego state EKF', True, {}),
        ('h3_driver', 'launch/h3.launch.py', 'H3 sensor', False, {}),  # Optional
    ]
    
    launch_descriptions = [declare_include_h3, declare_data_collection_mode]
    
    for package, relative_path, description, is_required, pass_args in launch_configs:
        if file_exists(package, relative_path):
            launch_desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare(package),
                        relative_path
                    ])
                ),
                launch_arguments=pass_args.items() if pass_args else None
            )
            
            # For optional sensors, add condition to launch argument
            if package == 'h3_driver':
                launch_desc = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare(package),
                            relative_path
                        ])
                    ),
                    launch_arguments=pass_args.items() if pass_args else None,
                    condition=include_h3
                )
            
            launch_descriptions.append(launch_desc)
        else:
            warning_msg = f"[WARNING] {relative_path} file does not exist in '{package}' package. Skipping {description}"
            print(warning_msg)
    
    return LaunchDescription(launch_descriptions)
