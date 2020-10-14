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
    waypoints = os.path.join(
        get_package_share_directory('omnivelma_navigation_2'),
        'params',
        'waypoints.yaml')
    return LaunchDescription([
        launch_ros.actions.Node(
            package="omnivelma_navigation_2",
            executable="waypoint_follower",
            name="waypoint_follower_node",
            output="screen",
            emulate_tty=True,
            parameters=[waypoints]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
