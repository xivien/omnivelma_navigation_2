import os
import sys

import launch
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_merged_tf_broadcaster',
            arguments=['0', '0.0', '0.176', '0.0', '0.0',
                       '0.0', 'omnivelma', 'base_laser'],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_r_tf_broadcaster',
            arguments=['0', '-0.3', '0.176', '3.141592', '0.0', '0.0',
                       'omnivelma', 'monokl_r_heart'],
            output='screen'),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_l_tf_broadcaster',
            arguments=['0', '0.3', '0.176', '0.0', '0.0',
                       '0.0', 'omnivelma', 'monokl_l_heart'],
            output='screen'),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
