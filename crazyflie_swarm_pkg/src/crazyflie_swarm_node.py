import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from crazyflie_swarm_interfaces.msg import SwarmPoses, CrazyfliePose

from script.crazyflie_swarm import CrazyflieSwarm


class CrazyflieSwarmNode(Node):
    def __init__(self):
        super().__init__('crazyflie_swarm_node')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        #* Parameters
        self.declare_parameter('uris', ['']) 
        self.declare_parameter('poses_rate_publisher', 10.0)
        
        self.uris = self.get_parameter('uris').get_parameter_value().string_array_value
        self.poses_rate_publisher = self.get_parameter('poses_rate_publisher').get_parameter_value().double_value
        
        self.get_logger().info(f'CrazyflieSwarmNode started with parameters:')
        self.get_logger().info(f' - uris: {self.uris}')
        self.get_logger().info(f' - poses_rate_publisher: {self.poses_rate_publisher}')
        
        #* Subscriptions
        self.leds_subscriber = self.create_subscription(Float32MultiArray, '/crazyflie_swarm/leds', self.leds_callback, 10)
        
        #* Publishers
        self.poses_publisher = self.create_publisher(SwarmPoses, '/crazyflie_swarm/poses', 10)
        self.poses_publisher_timer = self.create_timer(1/self.poses_rate_publisher, self.poses_callback)
        
        self.link_state_publisher = self.create_publisher(Bool, '/crazyflie_swarm/link_state', 10)
        self.link_state_publisher_timer = self.create_timer(1, self.link_state_callback)

        try:
            self.crazyflie_swarm = CrazyflieSwarm(self.uris)
            self.crazyflie_swarm.connect()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CrazyflieSwarm: {e}')
        
    def leds_callback(self, msg):    
        try:
            uri_0_intensity = int(msg.data[0])
            uri_1_intensity = int(msg.data[1])
            
            sequence_led_0 = [uri_0_intensity]
            sequence_led_1 = [uri_1_intensity]
        
            sequence = {
                self.uris[0]: [sequence_led_0],
                self.uris[1]: [sequence_led_1],
            } 

            self.crazyflie_swarm.command_leds(sequence)
            self.poses_callback()
        except Exception as e:
            self.get_logger().error(f'Error in leds_callback: {e}')
              
    def poses_callback(self):        
        try:
            positions = self.crazyflie_swarm.get_estimated_positions()
            euler_orientations = self.crazyflie_swarm.get_estimated_euler_orientations()
            quaternion_orientations = self.crazyflie_swarm.get_estimated_quaternions_orientations()

            swarm_poses_msg = SwarmPoses()

            for uri in self.uris:
                position = positions[uri]
                euler_orientation = euler_orientations[uri]
                quaternion_orientation = quaternion_orientations[uri]
                
                pose_msg = CrazyfliePose()
                pose_msg.uri = uri
                pose_msg.position = position
                pose_msg.euler_orientation = euler_orientation
                pose_msg.quaternion_orientation = quaternion_orientation
                
                swarm_poses_msg.poses.append(pose_msg)
            
            self.poses_publisher.publish(swarm_poses_msg)
        except Exception as e:
            self.get_logger().error(f'Error in poses_callback: {e}')
        
    def link_state_callback(self):
        try:
            link_state = self.crazyflie_swarm.swarm.are_links_open()
            msg = Bool()
            msg.data = link_state
            self.link_state_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in link_state_callback: {e}')
                          
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