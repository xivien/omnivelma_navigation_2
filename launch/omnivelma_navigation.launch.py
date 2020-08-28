import os
import sys

import launch
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config_dir = os.path.join(
            get_package_share_directory('omnivelma_navigation_2'),
            'rviz2',
            'rviz2_config.rviz')

    ld = launch.LaunchDescription([
        # Run another launch file
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/navigation.launch.py')
            ),
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/ekf.launch.py')
            ),
        ),
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen'),
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
