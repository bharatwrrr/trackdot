import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml
from datetime import datetime


def load_yaml(file_path):
    """Load and parse a YAML file."""
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)


def file_exists(package_name, relative_path):
    """Check if a file exists in a package."""
    try:
        package_dir = get_package_share_directory(package_name)
        full_path = os.path.join(package_dir, relative_path)
        return os.path.exists(full_path)
    except Exception:
        return False


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')
    
    # Load data collection configuration
    config_file = os.path.join(bringup_dir, 'config', 'data_collection_params.yaml')
    config = load_yaml(config_file)
    
    dc_config = config['data_collection']
    bag_config = dc_config['bag_record']
    
    # Launch arguments (can override from command line)
    output_dir = LaunchConfiguration(
        'output_dir',
        default=dc_config['output_dir']
    )
    bag_name = LaunchConfiguration(
        'bag_name',
        default=f"{dc_config['bag_name_prefix']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    )
    
    # List of launch files to include: (package, path, description)
    launch_configs = [
        ('bringup', 'launch/sensors_only.launch.py', 'Sensor drivers'),
        ('bringup', 'launch/transforms.launch.py', 'Transform tree'),
    ]
    
    launch_descriptions = [
        DeclareLaunchArgument('output_dir', default_value=dc_config['output_dir'],
                             description='Output directory for bag files'),
        DeclareLaunchArgument('bag_name', 
                             default_value=f"{dc_config['bag_name_prefix']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}",
                             description='Bag filename (without extension)'),
    ]
    
    # Include launch files if they exist
    for package, relative_path, description in launch_configs:
        if file_exists(package, relative_path):
            # Special handling for sensors_only — pass data_collection_mode: true
            launch_args = {}
            if 'sensors_only' in relative_path:
                launch_args = {'data_collection_mode': 'true'}
            
            launch_desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare(package),
                        relative_path
                    ])
                ),
                launch_arguments=launch_args.items() if launch_args else None
            )
            launch_descriptions.append(launch_desc)
        else:
            warning_msg = f"[WARNING] {relative_path} file does not exist in '{package}' package. Skipping {description}"
            print(warning_msg)
    
    # Resolve QoS overrides file path
    qos_overrides_file = os.path.join(
        bringup_dir,
        bag_config['qos_overrides_path']
    )
    
    # Build ros2 bag record command from config
    topics_to_record = bag_config['topics']
    compression_mode = bag_config['compression']['mode']
    compression_format = bag_config['compression']['format']
    
    mcap_config_file = os.path.join(bringup_dir, 'config', 'mcap_storage_config.yaml')

    bag_record_cmd = [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--storage-config-file', mcap_config_file,
#        '--qos-profile-overrides-path', qos_overrides_file,
        '-o', PathJoinSubstitution([output_dir, bag_name])
    ]
    # Add all topics from config
    bag_record_cmd.extend(topics_to_record)
    
    # Launch the bag recorder
    bag_recorder = ExecuteProcess(
        cmd=bag_record_cmd,
        output='screen',
        shell=False
    )
    
    launch_descriptions.append(bag_recorder)
    
    return LaunchDescription(launch_descriptions)
