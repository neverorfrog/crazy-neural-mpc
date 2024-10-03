import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CrazyflieDock(Node):
  def __init__(self):
    super().__init__('crazyflie_dock_node')
    self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
    #* Parameters
    self.declare_parameter('rate', 1.0)
    self.declare_parameter('crazyflie_robots.robot_1.name', 'crazyflie_robot_1')
    self.declare_parameter('crazyflie_robots.robot_1.uri', 'radio://0/80/2M/E7E7E7E7E7')
    
    self.declare_parameter('crazyflie_robots.robot_2.name', 'crazyflie_robot_2')
    self.declare_parameter('crazyflie_robots.robot_2.uri', 'radio://0/80/2M/E7E7E7E7E8')

    self.rate = self.get_parameter('rate').get_parameter_value().double_value
    
    self.crazyflie_params = {
      self.get_parameter('crazyflie_robots.robot_1.name').get_parameter_value().string_value : 
      {
        'uri': self.get_parameter('crazyflie_robots.robot_1.uri').get_parameter_value().string_value,
      },
      self.get_parameter('crazyflie_robots.robot_2.name').get_parameter_value().string_value : 
      {
        'uri': self.get_parameter('crazyflie_robots.robot_2.uri').get_parameter_value().string_value,
      },
    }
        
    self.get_logger().info(f'CrazyflieDockNode started with parameters:')
    self.get_logger().info(f'- crazyflie_params: ')
    for name, params in self.crazyflie_params.items():
      self.get_logger().info(f'  - {name}: {params}')
    
    self.led_publishers = {}
    for name, _ in self.crazyflie_params.items():
      publisher = self.create_publisher(Float32, f'/{name}/led', 10)
      self.led_publishers[name] = publisher
      self.create_timer(1/self.rate, lambda name=name, publisher=publisher: self.led_callback(name, publisher))
    
    self.k1 = 0
    self.k2 = 0
        
  def led_callback(self, name, publisher):
    intensity = Float32()
    if name == 'crazyflie_robot_1': 
      if self.k1%2==0: intensity.data = 255.0    
      else: intensity.data = 0.0
      self.k1 += 1
    elif name == 'crazyflie_robot_2': 
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