from typing import Dict
from script.config import SwarmConfig, load_config

from crazyflie_swarm_interfaces.msg import CrazyflieVelocity


import rclpy
from rclpy.node import Node, Publisher, Subscription
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class CrazyflieDock(Node):
  def __init__(self):
    super().__init__('crazyflie_dock_node')
    self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    self.get_logger().info(f'CrazyflieDockNode started')
    
    self.declare_parameter('package_path', '')
    root = self.get_parameter('package_path').get_parameter_value().string_value
    config = load_config(f'{root}/config/config.yaml', SwarmConfig)   
    self.config = config
          
    #* CrazyflieSwarm
    self.swarm: Dict[str] = {} 
    for crazyflie_config in self.config.crazyflies:
      uri = crazyflie_config.uri
      name = crazyflie_config.name
      self.swarm[name] = uri
    
    #* Publishers
    self.led_publishers: Dict[str, Publisher] = {}
    led_publisher_rate = self.config.led_publisher_rate
    for name, _ in self.swarm.items():
      publisher = self.create_publisher(Float32, f'/{name}/led', 10)
      self.led_publishers[name] = publisher
      self.create_timer(1/led_publisher_rate, lambda name=name, publisher=publisher: self.led_callback(name, publisher))
    
    self.velocity_publishers: Dict[str, Publisher] = {}
    velocity_publisher_rate = self.config.velocity_publisher_rate
    for name, _ in self.swarm.items():
      publisher = self.create_publisher(CrazyflieVelocity, f'/{name}/velocity', 10)
      self.velocity_publishers[name] = publisher
      # self.create_timer(1/velocity_publisher_rate, lambda name=name, publisher=publisher: self.velocity_callback(name, publisher))
      
    #* Subscriptions
    self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
    self.k1 = 0
    self.k2 = 0
    self.k3 = 0
        
  def led_callback(self, name: str, publisher: Publisher) -> None:
    intensity = Float32()
    if name == 'cf1': 
      if self.k1%2==0: intensity.data = 255.0    
      else: intensity.data = 0.0
      self.k1 += 1
    elif name == 'cf2': 
      if self.k2%2==0: intensity.data = 0.0    
      else: intensity.data = 255.0
      self.k2 += 1
    elif name == 'cf3': 
      if self.k3%2==0: intensity.data = 0.0    
      else: intensity.data = 255.0
      self.k3 += 1
    publisher.publish(intensity)
    
  def cmd_vel_callback(self, msg: Twist) -> None:
    cmd_vel = msg
    velocity_msg = CrazyflieVelocity()
    velocity_msg.header.stamp = self.get_clock().now().to_msg()
    velocity_msg.linear_velocity[0] = cmd_vel.linear.x
    velocity_msg.linear_velocity[1] = cmd_vel.linear.y
    velocity_msg.linear_velocity[2] = cmd_vel.linear.z
    velocity_msg.angular_velocity[0] = 0.0
    velocity_msg.angular_velocity[1] = 0.0
    velocity_msg.angular_velocity[2] = cmd_vel.angular.z
    for name, publisher in self.velocity_publishers.items():
      publisher.publish(velocity_msg)
    
    
def main(args=None):
    rclpy.init(args=args)
    crazyflie_node = CrazyflieDock()
    rclpy.spin(crazyflie_node)
    crazyflie_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()