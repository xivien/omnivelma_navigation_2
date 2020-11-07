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

    nav_params_dir = os.path.join(omnivelma_dir, 'params', 'nav2_params.yaml')
    bt_dir = os.path.join(omnivelma_dir, 'params', 'nav_bt.xml')

    nav_params_cmd = launch.actions.DeclareLaunchArgument(
        name='nav_params_file',
        default_value=nav_params_dir,
        description='Navigation parameters file'
    )

    bt_cmd = launch.actions.DeclareLaunchArgument(
        name='bt_file',
        default_value=bt_dir,
        description='Behaviour tree file'
    )

    autostart_cmd = launch.actions.DeclareLaunchArgument(
        name='autostart',
        default_value= 'true',
        description='Automatically startup the nav2 stack'
    )

    use_sim_time = launch.actions.DeclareLaunchArgument(
        name='use_sim_time',
        default_value= 'true',
        description='Use simulation (Gazebo) clock if true'
    )

    map_subscribe_transient_local_cmd = launch.actions.DeclareLaunchArgument(
        name='map_subscribe_transient_local',
        default_value= 'true',
        description='Whether to set the map subscriber QoS to transient local'
    )

    navigation_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': launch.substitutions.LaunchConfiguration('nav_params_file'),
            'default_bt_xml_filename': launch.substitutions.LaunchConfiguration('bt_file'),
            'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
            'autostart': launch.substitutions.LaunchConfiguration('autostart'),
            'map_subscribe_transient_local': launch.substitutions.LaunchConfiguration('map_subscribe_transient_local'),
        }.items(),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(nav_params_cmd)
    ld.add_action(bt_cmd)
    ld.add_action(autostart_cmd)
    ld.add_action(use_sim_time)
    ld.add_action(map_subscribe_transient_local_cmd)

    ld.add_action(navigation_cmd)

    return ld


if __name__ == '__main__':
    generate_launch_description()
