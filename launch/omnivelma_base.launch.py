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
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    rviz_config_dir = os.path.join(omnivelma_dir, 'rviz2/rviz2_config.rviz')

    nav_params_dir = os.path.join(omnivelma_dir, 'params', 'nav2_params.yaml')
    bt_dir = os.path.join(omnivelma_dir, 'params', 'nav_bt.xml')

    slam = LaunchConfiguration('use_slam')

    nav_params_cmd = launch.actions.DeclareLaunchArgument(
        name='nav_params_file',
        default_value=nav_params_dir,
        description="Navigation parameters file"
    )

    bt_cmd = launch.actions.DeclareLaunchArgument(
        name='bt_file',
        default_value=bt_dir,
        description="Behaviour tree file"
    )

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

    tf_broadcaster_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                omnivelma_dir, 'launch/laser_tf_broadcaster.launch.py')
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

    navigation_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': launch.substitutions.LaunchConfiguration('nav_params_file'),
            'default_bt_xml_filename': launch.substitutions.LaunchConfiguration('bt_file'),
            'use_sim_time': 'true',
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(use_slam_cmd)
    ld.add_action(nav_params_cmd)
    ld.add_action(bt_cmd)
    # ld.add_action(map_cmd)

    ld.add_action(rviz_cmd)
    ld.add_action(laserscan_merger_cmd)
    ld.add_action(tf_broadcaster_cmd)

    # EKF doesnt work in simulation
    ld.add_action(ekf_odom_cmd)
    # ld.add_action(ekf_map_cmd)

    # ld.add_action(amcl_cmd)
    # ld.add_action(slam_cmd)
    # ld.add_action(navigation_cmd)

    return ld


if __name__ == '__main__':
    generate_launch_description()
