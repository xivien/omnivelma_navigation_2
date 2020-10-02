import os
import sys

import launch
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition

def generate_launch_description():

    rviz_config_dir = os.path.join(
        get_package_share_directory('omnivelma_navigation_2'),
        'rviz2',
        'rviz2_config.rviz')
    slam = LaunchConfiguration('use_slam')

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_slam',
            default_value='False',
            description="SLAM or localization"
        ),
        # Run another launch file
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/navigation.launch.py')
            ),
            launch_arguments={
                'use_slam': launch.substitutions.LaunchConfiguration('use_slam')}.items(),
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/ekf_odom.launch.py')
            ),
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/ekf_map.launch.py')
            ),
            condition=IfCondition(PythonExpression(['not ', slam])),
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/laser_tf_broadcaster.launch.py')
            ),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
        # Node(
        #     package='omnivelma_navigation_2',
        #     executable='laserscan_multi_merger',
        #     name='laserscan_multi_merger',
        #     output='screen'),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
