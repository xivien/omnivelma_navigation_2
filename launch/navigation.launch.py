import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    map_location = os.path.join(
        get_package_share_directory('omnivelma_navigation_2'),
        'maps',
        'lab_updated.yaml')
    params_location = os.path.join(
        get_package_share_directory('omnivelma_navigation_2'),
        'params',
        'nav2_params.yaml')
    bt_xml_location = os.path.join(
        get_package_share_directory('omnivelma_navigation_2'),
        'params',
        'nav_bt.xml')

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='map',
            default_value=map_location,
            description="Map file path"
        ),
        launch.actions.DeclareLaunchArgument(
            name='params_file',
            default_value=params_location,
            description="Navigation parameters file"
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False', # True gives rlcpp errors??
            description="Navigation parameters file"
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time_slam',
            default_value='True',
            description="Navigation parameters file"
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_slam',
            default_value='False',
            description="SLAM or localization"
        ),
        launch.actions.DeclareLaunchArgument(
            name='bt_xml',
            default_value=bt_xml_location,
            description="Behaviour tree location"
        ),
        # Run another launch file
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'omnivelma_navigation_2'), 'launch/bringup_launch.py')
            ),
            launch_arguments={
                'map': launch.substitutions.LaunchConfiguration('map'),
                'params_file': launch.substitutions.LaunchConfiguration('params_file'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'),
                'use_sim_time_slam': launch.substitutions.LaunchConfiguration('use_sim_time_slam'),
                'slam': launch.substitutions.LaunchConfiguration('use_slam'),
                'default_bt_xml_filename': launch.substitutions.LaunchConfiguration('bt_xml'),
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
