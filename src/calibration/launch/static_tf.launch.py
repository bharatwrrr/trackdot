from launch import LaunchDescription
from launch_ros.actions import Node
import yaml
import os

def load_tf_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
        key = next(iter(data))
        return data[key]

def generate_launch_description():

    pkg_share = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        './../../install/calibration/share/calibration/config'
    )
    print(f"Package share path: {pkg_share}")
    # Load configs
    h3 = load_tf_yaml(os.path.join(pkg_share, 'h3_tf.yaml'))
    triton = load_tf_yaml(os.path.join(pkg_share, 'triton_left_tf.yaml'))

    return LaunchDescription([

        # LIDAR TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='h3_static_tf',
            arguments=[
                str(h3['x']),
                str(h3['y']),
                str(h3['z']),
                str(h3['roll']),
                str(h3['pitch']),
                str(h3['yaw']),
                h3['parent'],
                h3['child']
            ]
        ),

        # Camera TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='triton_left_static_tf',
            arguments=[
                str(triton['x']),
                str(triton['y']),
                str(triton['z']),
                str(triton['roll']),
                str(triton['pitch']),
                str(triton['yaw']),
                triton['parent'],
                triton['child']
            ]
        ),
    ])
