from typing import Any, Dict

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node, Publisher, Subscription
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from crazyflie_swarm_interfaces.msg import CrazyflieState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from crazyflie_simulation_pkg.utils import SwarmConfig, load_config
from crazyflie_swarm_pkg.crazyflie.crazyflie_state import CrazyState


class CrazyflieSimulation(Node):
    def __init__(self):
        super().__init__("crazyflie_simulation_node")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # * Load Config
        self.declare_parameter("swarm_config_path", "")
        swarm_config_path = (
            self.get_parameter("swarm_config_path")
            .get_parameter_value()
            .string_value
        )
        config = load_config(swarm_config_path, SwarmConfig)
        self.config = config

        self.get_logger().info("CrazyflieSimulationNode started")
        for cf_config in self.config.crazyflies:
            self.get_logger().info(f"  - {cf_config.name}")

        # * CrazyflieSwarm
        self.swarm = []
        for crazyflie_config in self.config.crazyflies:
            if crazyflie_config.active is False:
                continue
            name = crazyflie_config.name
            self.swarm.append(name)

        self.get_logger().info(f"Swarm: {self.swarm}")

        # * Publishers
        self.velocity_publishers: Dict[str, Publisher] = {}
        velocity_publisher_rate = self.config.velocity_publisher_rate
        for name in self.swarm:
            publisher = self.create_publisher(Twist, f"/{name}/cmd_vel", 10)
            self.velocity_publishers[name] = publisher
            self.create_timer(
                1 / velocity_publisher_rate,
                lambda name=name, publisher=publisher: self.velocity_callback(
                    name, publisher
                ),
            )
            
        self.state_publishers: Dict[str, Publisher] = {}
        state_publisher_rate = self.config.state_publisher_rate
        for name in self.swarm:
            publisher = self.create_publisher(CrazyflieState, f"/{name}/state", 10)
            self.state_publishers[name] = publisher
            self.create_timer(
                1 / state_publisher_rate,
                lambda name=name, publisher=publisher: self.state_callback(
                    name, publisher
                ),
            )

        # * Subscriptions
        self.current_odoms: Dict[str, Odometry] = {}
        for name in self.swarm:
            self.current_odoms[name] = Odometry()
        
        self.odom_subscribers: Dict[str, Subscription] = {}
        for name in self.swarm:
            self.odom_subscribers[name] = self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, name=name: self.odom_callback(msg, name),
                10,
            )
        
        self.current_multirangers: Dict[str, LaserScan] = {}
        for name in self.swarm:
            self.current_multirangers[name] = LaserScan()
        
        self.multiranger_subscribers: Dict[str, Subscription] = {}
        for name in self.swarm:
            self.multiranger_subscribers[name] = self.create_subscription(
                LaserScan,
                f"/{name}/multiranger",
                lambda msg, name=name: self.multiranger_callback(msg, name),
                10,
            )
        
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        self.current_cmd_vel = Twist()

        self.current_states: Dict[str, CrazyState] = {}
        for name in self.swarm:
            self.current_states[name] = CrazyState()
            
        self.max_ang_z_rate = self.config.max_ang_z_rate
        self.takeoff_height = self.config.height
        self.takeoff_command = False
        self.is_flying = False
        self.keep_height = False

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.current_cmd_vel = msg

    def odom_callback(self, msg: Odometry, name: str) -> None:
        self.current_odoms[name] = msg
        state = CrazyState()
        
        state.x = self.current_odoms[name].pose.pose.position.x
        state.y = self.current_odoms[name].pose.pose.position.y
        state.z = self.current_odoms[name].pose.pose.position.z
        state.roll = self.current_odoms[name].pose.pose.orientation.x
        state.pitch = self.current_odoms[name].pose.pose.orientation.y
        state.yaw = self.current_odoms[name].pose.pose.orientation.z
        state.vx = self.current_odoms[name].twist.twist.linear.x
        state.vy = self.current_odoms[name].twist.twist.linear.y
        state.vz = self.current_odoms[name].twist.twist.linear.z
        state.roll_rate = self.current_odoms[name].twist.twist.angular.x
        state.pitch_rate = self.current_odoms[name].twist.twist.angular.y
        state.yaw_rate = self.current_odoms[name].twist.twist.angular.z
        state.mr_front = self.current_multirangers[name].ranges[0]
        state.mr_right = self.current_multirangers[name].ranges[1]
        state.mr_back = self.current_multirangers[name].ranges[2]
        state.mr_left = self.current_multirangers[name].ranges[3]
        
        self.current_states[name] = state
        
    def multiranger_callback(self, msg: LaserScan, name: str) -> None:   
        self.current_multirangers[name] = msg
        
    def state_callback(self, name: str, publisher: Publisher) -> None:
        state_msg = CrazyflieState()
        state = self.current_states[name]
        state_msg.position[0] = state.x
        state_msg.position[1] = state.y
        state_msg.position[2] = state.z
        state_msg.euler_orientation[0] = state.roll
        state_msg.euler_orientation[1] = state.pitch
        state_msg.euler_orientation[2] = state.yaw
        state_msg.linear_velocity[0] = state.vx
        state_msg.linear_velocity[1] = state.vy
        state_msg.linear_velocity[2] = state.vz
        state_msg.angular_velocity[0] = state.roll_rate
        state_msg.angular_velocity[1] = state.pitch_rate
        state_msg.angular_velocity[2] = state.yaw_rate
        state_msg.multiranger[0] = state.mr_front
        state_msg.multiranger[1] = state.mr_right
        state_msg.multiranger[2] = state.mr_back
        state_msg.multiranger[3] = state.mr_left
        
        publisher.publish(state_msg)

    def velocity_callback(self, name: str, publisher: Publisher) -> None:
        current_velocity_msg = self.current_cmd_vel
        height_command = current_velocity_msg.linear.z
        new_velocity_msg = Twist()

        # If the drone is flying, only allow to transfer the twist message
        if self.is_flying:
            new_velocity_msg.linear.x = current_velocity_msg.linear.x
            new_velocity_msg.linear.y = current_velocity_msg.linear.y
            new_velocity_msg.linear.z = current_velocity_msg.linear.z
            new_velocity_msg.angular.x = current_velocity_msg.angular.x
            new_velocity_msg.angular.y = current_velocity_msg.angular.y
            new_velocity_msg.angular.z = current_velocity_msg.angular.z

        # If not flying and receiving a velocity height command, takeoff
        if height_command > 0 and not self.is_flying:
            new_velocity_msg.linear.z = 0.5
            if self.current_states[name].z > self.takeoff_height:
                # stop going up if height is reached
                new_velocity_msg.linear.z = 0.0
                self.current_cmd_vel.linear.z = 0.0
                self.is_flying = True
                self.get_logger().info('Takeoff completed')

        # If flying and if the height command is negative, and it is below a certain height
        # then consider it a land
        if height_command < 0 and self.is_flying:
            if self.current_states[name].z < 0.1:
                new_velocity_msg.linear.z = 0.0
                self.is_flying = False
                self.keep_height = False
                self.get_logger().info('Landing completed')

        # Cap the angular rate command in the z axis
        if abs(current_velocity_msg.angular.z) > self.max_ang_z_rate:
            new_velocity_msg.angular.z = self.max_ang_z_rate * abs(current_velocity_msg.angular.z)/current_velocity_msg.angular.z

        # If there is no control in height and the drone is flying, control and maintain the height
        tolerance = 1e-7
        if abs(height_command) < tolerance and self.is_flying:
            if not self.keep_height:
                self.desired_height = self.current_states[name].z
                self.keep_height = True
            else:
                error = self.desired_height - self.current_states[name].z
                new_velocity_msg.linear.z = error

        # If there is control in height and the drone is flying, stop maintaining the height
        if abs(height_command) > tolerance and self.is_flying:
            if self.keep_height:
                self.keep_height = False

        publisher.publish(new_velocity_msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    crazyflie_node = CrazyflieSimulation()
    rclpy.spin(crazyflie_node)
    crazyflie_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
