"""bringup.launch.py - Lanza el driver del Yahboom MicroROS-Pi5.

Publica: /odom, /tf, /scan
Suscribe: /cmd_vel
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('capytown_esan'),
        'config'
    )

    wheel_params = os.path.join(config_dir, 'wheel_params.yaml')

    return LaunchDescription([
        Node(
            package='yahboom_driver',
            executable='yahboom_node',
            name='yahboom_driver',
            parameters=[wheel_params],
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        )
    ])
