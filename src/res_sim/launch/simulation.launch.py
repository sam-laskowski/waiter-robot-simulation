import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    pkg_res_sim = get_package_share_directory('res_sim')

    simulation_node = Node(
        package="res_sim",
        executable="robot_controller",
        name="robot_controller",
    )

    ld = LaunchDescription()

    ld.add_action(simulation_node)

    return ld