#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('capytown_esan')

    hsv_params = os.path.join(pkg_share, 'config', 'hsv_params.yaml')
    pid_params = os.path.join(pkg_share, 'config', 'pid_params.yaml')

    lane_detector = Node(
        package='capytown_esan',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
        parameters=[hsv_params]
    )

    lane_controller = Node(
        package='capytown_esan',
        executable='lane_controller',
        name='lane_controller',
        output='screen',
        parameters=[pid_params]
    )

    return LaunchDescription([
        lane_detector,
        lane_controller
    ])
