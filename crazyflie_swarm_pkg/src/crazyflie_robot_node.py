import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from crazyflie_swarm_interfaces.msg import CrazyfliePose

from script.swarm import CrazyflieRobot

class CrazyflieRobotNode(Node):
    def __init__(self):
        super().__init__('crazyflie_robot_node', allow_undeclared_parameters=True)
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        #* Parameters
        self.declare_parameter('name', '') 
        self.declare_parameter('uri', '') 
        self.declare_parameter('pose_rate_publisher', 10.0)
        
        self.robot = self.get_parameter('robot').value
        self.name = self.get_parameter('name').get_parameter_value().string_value
        self.uri = self.get_parameter('uri').get_parameter_value().string_value
        self.poses_rate_publisher = self.get_parameter('pose_rate_publisher').get_parameter_value().double_value
        
        self.get_logger().info(f'CrazyflieRobotNode started with parameters:')
        self.get_logger().info(f' - name: {self.name}')
        self.get_logger().info(f' - uri: {self.uri}')
        self.get_logger().info(f' - pose_rate_publisher: {self.poses_rate_publisher}')
        
        #* Subscriptions
        self.leds_subscriber = self.create_subscription(Float32, f'/crazyflie_robot/{self.name}/led', self.led_callback, 10)
        
        #* Publishers
        self.poses_publisher = self.create_publisher(CrazyfliePose, f'/crazyflie_robot/{self.name}/pose', 10)
        self.poses_publisher_timer = self.create_timer(1/self.poses_rate_publisher, self.pose_callback)
        
        #* CrazyflieRobot
        try:
            self.crazyflie_robot = CrazyflieRobot(self.uri, ro_cache='./ro_cache', rw_cache='./rw_cache')
            while not self.crazyflie_robot.initialize():
                print('Connecting to Crazyflies ...')
                time.sleep(0.5)
            print('Connected to Crazyflies')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to CrazyflieSwarm: {e}')
        
    def led_callback(self, msg):    
        self.get_logger().info(f'[{self.name}][led_callback]: {msg.data}')
        try:
            intensity = int(msg.data)     
            self.crazyflie_robot.set_led(intensity) 
        except Exception as e:
            self.get_logger().error(f'[{self.name}][led_callback]: {e}')
              
    def pose_callback(self):        
        try:
            position = self.crazyflie_robot.get_estimated_position()
            euler_orientation = self.crazyflie_robot.get_estimated_euler_orientation()
            quaternion_orientation = self.crazyflie_robot.get_estimated_quaternion_orientation()
            
            pose_msg = CrazyfliePose()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.uri = self.uri
            pose_msg.position[0] = position['x']
            pose_msg.position[1] = position['y']
            pose_msg.position[2] = position['z']
            pose_msg.euler_orientation[0] = euler_orientation['roll']
            pose_msg.euler_orientation[1] = euler_orientation['pitch']
            pose_msg.euler_orientation[2] = euler_orientation['yaw']
            pose_msg.quaternion_orientation[0] = quaternion_orientation['qx']
            pose_msg.quaternion_orientation[1] = quaternion_orientation['qy']
            pose_msg.quaternion_orientation[2] = quaternion_orientation['qz']
            pose_msg.quaternion_orientation[3] = quaternion_orientation['qw']
            
            self.poses_publisher.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f'[{self.name}][pose_callback]: {e}')
        
                                  
def main(args=None):
    rclpy.init(args=args)
    crazyflie_swarm_node = CrazyflieRobotNode()
    try:
        rclpy.spin(crazyflie_swarm_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        crazyflie_swarm_node.get_logger().error(f'Error in main loop: {e}')
    finally:
        crazyflie_swarm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()