from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os
from launch_ros.actions import Node


def generate_launch_description():

    turtlebot_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    "turtlebot3_gazebo"
                ),
                "launch",
                "turtlebot3_world.launch.py"
            )
        )
    )
    rviz = Node(
    package="rviz2",
    executable="rviz2",
    output="screen"
)

    return LaunchDescription([
        turtlebot_world,
        rviz
    ])