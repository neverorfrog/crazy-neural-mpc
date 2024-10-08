from typing import Dict
from script.crazyflie_swarm import CrazyflieSwarm
import rclpy
from rclpy.node import Node, Publisher, Subscription
from std_msgs.msg import Float32

class CrazyflieDock(Node):
  def __init__(self):
    super().__init__('crazyflie_dock_node')
    self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    self.get_logger().info(f'CrazyflieDockNode started')
    
    #* CrazyflieSwarm (with config inside)
    self.swarm = CrazyflieSwarm(ros2_logger=self.get_logger())
    
    #* Publishers
    self.led_publishers: Dict[str, Publisher] = {}
    led_publisher_rate = self.swarm.config.led_publisher_rate
    for name, _ in self.swarm:
      publisher = self.create_publisher(Float32, f'/{name}/led', 10)
      self.led_publishers[name] = publisher
      self.create_timer(1/led_publisher_rate, lambda name=name, publisher=publisher: self.led_callback(name, publisher))
        
    self.k1 = 0
    self.k2 = 0
        
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
    publisher.publish(intensity)
    
def main(args=None):
    rclpy.init(args=args)
    crazyflie_node = CrazyflieDock()
    rclpy.spin(crazyflie_node)
    crazyflie_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()