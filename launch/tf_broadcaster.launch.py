import os
import sys

import launch
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# static broadcaster arguments x y z yaw pitch roll frame_id child_frame_id


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
        Node(  # Camera tf broadcaster - base transform is changing and is published by gazebo
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_broadcaster',
            arguments=['0', '0.0', '0.0', '1.570796', '3.141592',
                       '1.570796', 'camera_link_reoriented', 'camera_link'],
            output='screen'),
        Node( # base transform is changing and is published by gazebo
            package='tf2_ros',
            executable='static_transform_publisher',
            name='omnivelma_tf_broadcaster',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0',
                       '0.0', 'base_footprint', 'omnivelma'],
            output='screen'),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
