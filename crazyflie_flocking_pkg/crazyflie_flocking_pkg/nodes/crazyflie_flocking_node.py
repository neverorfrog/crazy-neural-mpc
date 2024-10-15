from typing import Any, Dict

from crazyflie_swarm_pkg.crazyflie_state import CrazyState
import rclpy
from crazyflie_flocking_pkg.agent import Agent
from crazyflie_flocking_pkg.crazyflie_flocking_pkg.utils.configuration import FlockingConfig
from geometry_msgs.msg import Twist
from rclpy.node import Node, Publisher, Subscription

from crazyflie_swarm_pkg.configuration import SwarmConfig
from crazyflie_swarm_pkg.utils import load_config
from crazyflie_swarm_interfaces.msg import CrazyflieState


class CrazyflieFlocking(Node):  # type: ignore
    def __init__(self):
        super().__init__("crazyflie_dock_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # * Load Config
        self.declare_parameter("swarm_config_path", "")
        swarm_config_path = (
            self.get_parameter("swarm_config_path")
            .get_parameter_value()
            .string_value
        )
        swarm_config = load_config(swarm_config_path, SwarmConfig)
        self.swarm_config = swarm_config

        self.declare_parameter("flocking_config_path", "")
        flocking_config_path = (
            self.get_parameter("flocking_config_path")
            .get_parameter_value()
            .string_value
        )
        flocking_config = load_config(flocking_config_path, FlockingConfig)
        self.flocking_config = flocking_config

        self.get_logger().info("CrazyflieFlockingNode started")
        for cf_config in self.swarm_config.crazyflies:
            self.get_logger().info(f"  - {cf_config.name}: {cf_config.uri}")
        self.get_logger().info("Flocking parameters:")
        self.get_logger().info(f"  - {self.flocking_config}")

        # * CrazyflieSwarm (just the parameters)
        self.swarm: Dict[str] = {}
        for crazyflie_config in self.swarm_config.crazyflies:
            uri = crazyflie_config.uri
            name = crazyflie_config.name
            self.swarm[name] = uri

        # * Flocking Agents
        self.agents: Dict[str, Agent] = {}
        for name, _ in self.swarm.items():
            agent = Agent(name, self.flocking_config)
            self.agents[name] = agent

        # * Publishers
        self.cmd_vel_publishers: Dict[str, Publisher] = {}
        velocity_publisher_rate = self.swarm_config.velocity_publisher_rate
        for name, _ in self.swarm.items():
            publisher = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            self.cmd_vel_publishers[name] = publisher
            self.create_timer(
                1 / velocity_publisher_rate,
                lambda name=name, publisher=publisher: self.cmd_vel_callback(
                    name, publisher
                ),
            )

        # * Subscriptions
        self.state_subscribers: Dict[str, Subscription] = {}
        for name, _ in self.swarm.items():
            subscriber = self.create_subscription(
                CrazyflieState,
                f"/{name}/state",
                lambda msg, name=name: self.state_callback(msg, name),
                10,
            )
            self.state_subscribers[name] = subscriber
        self.swarm_state: Dict[str, CrazyState] = {}

    def cmd_vel_callback(self, name: str, publisher: Publisher) -> None:
        """
        Callback function for the cmd_vel publisher. Takes the desired
        velocities computed by the flocking algorithm and sends them to the dock
        node.
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        publisher.publish(cmd_vel)

    def state_callback(self, msg: CrazyflieState, name: str) -> None:
        """
        Callback function for the state subscriber. Subscribes to the states of
        the crazyflies in the swarm and saves them in the swarm_state dictionary.
        """
        self.swarm_state[name] = msg


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    crazyflie_node = CrazyflieFlocking()
    rclpy.spin(crazyflie_node)
    crazyflie_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
