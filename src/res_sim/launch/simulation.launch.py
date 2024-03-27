import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    pkg_res_sim = get_package_share_directory('res_sim')

    robot_controller = Node(
        package="res_sim",
        executable="robot_controller",
        name="robot_controller",
    )

    table_behaviour = Node(
        package="res_sim",
        executable="table",
        name="table"
    )

    ld = LaunchDescription()

    ld.add_action(robot_controller)

    return ld