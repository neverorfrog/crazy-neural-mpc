import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from crazyflie_swarm_interfaces.msg import SwarmPoses, CrazyfliePose

import cflib.crtp as crtp
from script.swarm import CrazyflieRobot

class CrazyflieSwarmNode(Node):
    def __init__(self):
        super().__init__('crazyflie_swarm_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        #* Parameters
        self.declare_parameter('robot_1', ['crazyflie_robot_1', 'radio://0/80/2M/E7E7E7E7E7'])
        self.declare_parameter('robot_2', ['crazyflie_robot_2', 'radio://0/80/2M/E7E7E7E7E7'])
        
        self.robot_1 = self.get_parameter('robot_1').get_parameter_value().string_array_value
        self.robot_2 = self.get_parameter('robot_2').get_parameter_value().string_array_value
            
        self.get_logger().info(f'CrazyflieDockNode started with parameters:')
        self.get_logger().info(f'- robots: ')
        self.get_logger().info(f'  - robot_1: {self.robot_1}')
        self.get_logger().info(f'  - robot_2: {self.robot_2}')
 
        self.robots = {
            self.robot_1[0]: self.robot_1[1],
            self.robot_2[0]: self.robot_2[1]
        }
        
        #* Subscriptions
        self.led_subscribers = {}
        for name, uri in self.robots.items():
            self.led_subscribers[name] = self.create_subscription(Float32, 
                                                                  f'/crazyflie_robot/{name}/led', 
                                                                  lambda msg, name=name: self.led_callback(msg, name), 
                                                                  10)
        
        #* CrazyflieSwarm
        crtp.init_drivers()
        self.crazyflie_swarm = {}        
        for name, uri in self.robots.items():
            crazyflie_robot = CrazyflieRobot(uri, ro_cache='./ro_cache', rw_cache='./rw_cache')
            while not crazyflie_robot.initialize():
                print(f'Connecting to Crazyflie {name} ...')
                time.sleep(0.5)
            print(f'Connected to Crazyflie {name}')
            self.crazyflie_swarm[name] = crazyflie_robot
        
    def led_callback(self, msg, name):    
        self.get_logger().info(f'Received message: {msg.data} for robot: {name}')
        self.crazyflie_swarm[name].set_led(int(msg.data))
              
                                              
def main(args=None):
    rclpy.init(args=args)
    crazyflie_swarm_node = CrazyflieSwarmNode()
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