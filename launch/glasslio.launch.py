import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = os.path.join(
    get_package_share_directory('glasslio'), 'config', 'livox_mid_360.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=DEFAULT_CONFIG,
            description='Sensor parameter file (topics, extrinsic, voxel size).',
        ),
        Node(
            package='glasslio',
            executable='glasslio_node',
            name='glasslio_node',
            output='screen',
            parameters=[LaunchConfiguration('config')],
        ),
    ])
