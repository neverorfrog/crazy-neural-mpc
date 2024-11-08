import rclpy
from rclpy.node import Node
from crazyflie_swarm_interfaces.msg import CrazyflieState
from crazyflie_swarm_pkg.crazyflie.crazyflie_state import CrazyState
import matplotlib.pyplot as plt
from crazyflie_plotter_pkg.utils import World
from crazyflie_plotter_pkg.utils import SwarmConfig, load_config
from crazyflie_flocking_pkg.utils.configuration import FlockingConfig

from ament_index_python.packages import get_package_share_directory
import numpy as np

import yaml
from omegaconf import OmegaConf

import os

root = get_package_share_directory("crazyflie_plotter_pkg")
file_path = os.path.join(root, "config", "msg_config.yaml")

with open(file_path, 'r') as file:
    config = OmegaConf.create(yaml.safe_load(file))

class PlotNode(Node):
    def __init__(self):
        super().__init__('plot_node')

        # * Load Config
        self.declare_parameter("flocking_config_path", "")
        flocking_config_path = (
            self.get_parameter("flocking_config_path")
            .get_parameter_value()
            .string_value
        )
        flocking_config = load_config(flocking_config_path, FlockingConfig)
        self.flocking_config = flocking_config

        self.declare_parameter("swarm_config_path", "")
        swarm_config_path = (
            self.get_parameter("swarm_config_path")
            .get_parameter_value()
            .string_value
        )
        swarm_config = load_config(swarm_config_path, SwarmConfig)
        self.config = swarm_config

        self.max_ang_z_rate = self.config.max_ang_z_rate
        self.takeoff_height = self.config.height

        # * CrazyflieSwarm
        self.swarm = []
        for crazyflie_config in self.config.crazyflies:
            if crazyflie_config.active is False:
                continue
            name = crazyflie_config.name
            self.swarm.append(name)

        self.states = {}
        self.drones = {}

        for name in self.swarm:
            self.subscription = self.create_subscription(
                CrazyflieState, f'/{name}/state', lambda msg, name=name: self.listener_callback(msg, name), 10
            )
            self.states[name] = CrazyState()
            self.drones[name] = self.states[name].toDrone(name, config)

        self.world = World(config.view_2d, config.plot_graphs, flocking_config)
        self.get_logger().info("CrazyfliePlotNode started")

    def listener_callback(self, msg, name):        
        state = CrazyState().fromMsg(msg)

        self.states[name] = state
        self.update_plot(name)

    def update_plot(self, name):
        #self.get_logger().info(f"Data: {self.states[name][-1]}")

        # Moving to the plotting convention - state -> drone
        self.drones[name] = self.states[name].toDrone(name, config)
        
        self.world.plot2d(self.drones) #, title = "t = " + str(np.round(robot.getTime(), 2)) + "s")

def main(args=None):
    rclpy.init(args=args)
    node = PlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
