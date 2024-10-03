import time

# ROS 2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class CrazyflieDock(Node):
    def __init__(self):
        super().__init__('crazyflie_dock_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        self.declare_parameter('rate', 1.0) 
        
        self.rate = self.get_parameter("rate").get_parameter_value().double_value
        
        self.get_logger().info(f'CrazyflieDockNode started with parameters:')
        self.get_logger().info(f' - rate: {self.rate}')
 
        self.leds_publisher = self.create_publisher(Float32MultiArray, '/crazyflie_swarm/leds', 10)
        self.leds_publisher_timer = self.create_timer(1/self.rate, self.publisher_callback)
        self.k = 0

    def publisher_callback(self):
        msg = Float32MultiArray()
        if self.k==0: 
            msg.data = [0.0, 255.0]
            self.k=1
        else: 
            msg.data = [255.0, 0.0]
            self.k=0
        self.leds_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    crazyflie_node = CrazyflieDock()
    rclpy.spin(crazyflie_node)
    crazyflie_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()