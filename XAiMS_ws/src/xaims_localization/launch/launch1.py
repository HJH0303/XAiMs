#!/usr/bin/env python3
# Author: Bishop Pearson
# Author: ChangWhan Lee
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  ekf_parameter = LaunchConfiguration(
    'ekf_parameter',
    default=os.path.join(
      get_package_share_directory('xaims_localization'),
      'params/ekf3.yaml'
    )
  ),

  return LaunchDescription([
    # Start robot localization using an Extended Kalman filter
    Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_parameter],
        emulate_tty=True,
    ),
    
    Node(
        package='xaims_localization',
        executable='ekf_odom_pub',
        name='ekf_odom_pub',
        output='screen',
        emulate_tty=True,

    ),
    Node(
        package='xaims_localization',
        executable='rviz_click_to_2d',
        name='rviz_click_to_2d',
        output='screen',
        emulate_tty=True,
    ),
    Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        emulate_tty=True,

    )
  ])
