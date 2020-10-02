from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ekf_odom_params = os.path.join(
        get_package_share_directory('omnivelma_navigation_2'),
        'params',
        'ekf_odom.yaml')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_odom_node',
            output='screen',
            parameters=[ekf_odom_params],
        ),
    ])
