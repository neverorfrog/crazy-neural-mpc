import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    root = get_package_share_directory("crazyflie_mpc_pkg")

    dataset_node = Node(
        package="crazyflie_mpc_pkg",
        executable="dataset_creation_exec",
        parameters=[{"mpc_config_path": os.path.join(root, "config/config.yaml"), "bags_path": os.path.join(root, "bags")}],
    )

    ld.add_action(dataset_node)

    return ld
