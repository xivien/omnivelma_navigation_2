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
    map_location = os.path.join(omnivelma_dir, 'maps', 'willow_garage.yaml')
    nav_params_dir = os.path.join(omnivelma_dir, 'params', 'nav2_params.yaml')

    slam = LaunchConfiguration('use_slam')

    nav_params_cmd = launch.actions.DeclareLaunchArgument(
        name='nav_params_file',
        default_value=nav_params_dir,
        description="Navigation parameters file"
    )

    map_cmd = launch.actions.DeclareLaunchArgument(
        name='map_file',
        default_value=map_location,
        description="Map file"
    )

    use_slam_cmd = DeclareLaunchArgument(
        name='use_slam',
        default_value='False',
        description="SLAM or localization"
    )

    slam_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(omnivelma_dir, 'launch/slam_launch.py')
        ),
        condition=IfCondition(slam),
        launch_arguments={
            # 'params_file': launch.substitutions.LaunchConfiguration('nav_params_file'),
        }.items(),

    )

    amcl_cmd = IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'localization_launch.py')
        ),
        condition=IfCondition(PythonExpression(['not ', slam])),
        launch_arguments={
            'params_file': launch.substitutions.LaunchConfiguration('nav_params_file'),
            'map': launch.substitutions.LaunchConfiguration('map_file')
        }.items(),

    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(use_slam_cmd)
    ld.add_action(nav_params_cmd)
    ld.add_action(map_cmd)

    ld.add_action(amcl_cmd)
    ld.add_action(slam_cmd)

    return ld


if __name__ == '__main__':
    generate_launch_description()
