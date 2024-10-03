import time
import yaml

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class CrazyflieDock(Node):
    def __init__(self):
        super().__init__('crazyflie_dock_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('robot_1', ['crazyflie_robot_1', 'radio://0/80/2M/E7E7E7E7E7'])
        self.declare_parameter('robot_2', ['crazyflie_robot_2', 'radio://0/80/2M/E7E7E7E7E7'])

        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self.robot_1 = self.get_parameter('robot_1').get_parameter_value().string_array_value
        self.robot_2 = self.get_parameter('robot_2').get_parameter_value().string_array_value
            
        self.get_logger().info(f'CrazyflieDockNode started with parameters:')
        self.get_logger().info(f'- robots: ')
        self.get_logger().info(f'  - robot_1: {self.robot_1}')
        self.get_logger().info(f'  - robot_2: {self.robot_2}')
        self.get_logger().info(f' - rate: {self.rate}')
 
        self.robots = {
            self.robot_1[0]: self.robot_1[1],
            self.robot_2[0]: self.robot_2[1]
        }
        
        self.led_publishers = {}
        for name, uri in self.robots.items():
            self.led_publishers[name] = self.create_publisher(Float32, f'/crazyflie_robot/{name}/led', 10)
        
        self.led_publishers_timer = self.create_timer(1/self.rate, self.led_publishers_callback)
        
        self.k = 0
        
    def led_publishers_callback(self):        
        intensity = Float32()
        if self.k%2 == 0: intensity.data = 0.0
        else: intensity.data = 255.0
        self.k += 1   

        for name, publisher in self.led_publishers.items():
            publisher.publish(intensity)
        

def main(args=None):
    rclpy.init(args=args)
    crazyflie_node = CrazyflieDock()
    rclpy.spin(crazyflie_node)
    crazyflie_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()