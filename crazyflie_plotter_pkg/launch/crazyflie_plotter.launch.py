# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    root = get_package_share_directory("crazyflie_simulation_pkg")
    gz_model_path = os.getenv("GZ_SIM_RESOURCE_PATH")
    if gz_model_path is None:
        raise EnvironmentError("GZ_SIM_RESOURCE_PATH is not set.")

    plotter = Node(
        package="crazyflie_simulation_pkg",
        executable="crazyflie_plot_exec",
        output='screen',
        parameters=[
            {"swarm_config_path": os.path.join(root, "config/config.yaml")},
        ],
    )

    return LaunchDescription([plotter])
