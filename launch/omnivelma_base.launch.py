import os
import sys

import launch
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    omnivelma_dir = get_package_share_directory('omnivelma_navigation_2')
    rviz_config_dir = os.path.join(omnivelma_dir, 'rviz2/rviz2_config.rviz')

    slam = LaunchConfiguration('use_slam')



    use_slam_cmd = DeclareLaunchArgument(
        name='use_slam',
        default_value='False',
        description="SLAM or localization"
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        # output='screen'
    )

    laserscan_merger_cmd = Node(
        package='omnivelma_navigation_2',
        executable='laserscan_multi_merger',
        name='laserscan_multi_merger',
        output='screen')
    
    error_pub_cmd = Node(
        package='omnivelma_navigation_2',
        executable='error_publisher',
        name='error_publisher',
        output='screen')
    
    cov_pub_cmd = Node(
        package='omnivelma_navigation_2',
        executable='cov_publisher',
        name='cov_publisher',
        output='screen')

    tf_broadcaster_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                omnivelma_dir, 'launch/tf_broadcaster.launch.py')
        ),
    )

    ekf_odom_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(omnivelma_dir, 'launch/ekf_odom.launch.py')
        ),
    )

    ekf_map_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(omnivelma_dir, 'launch/ekf_map.launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(use_slam_cmd)

    ld.add_action(rviz_cmd)
    ld.add_action(laserscan_merger_cmd)
    ld.add_action(error_pub_cmd)
    ld.add_action(cov_pub_cmd)
    ld.add_action(tf_broadcaster_cmd)

    ld.add_action(ekf_odom_cmd)
    ld.add_action(ekf_map_cmd)

    return ld


if __name__ == '__main__':
    generate_launch_description()
