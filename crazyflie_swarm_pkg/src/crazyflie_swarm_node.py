import time
from typing import Dict

import rclpy
from rclpy.node import Node, Subscription, Publisher
from std_msgs.msg import Float32

from crazyflie_swarm_interfaces.msg import CrazyflieState
from crazyflie_swarm_interfaces.srv import TakeOff, Land

from script.config import SwarmConfig, load_config
from script.crazyflie_robot import CrazyflieRobot
import cflib.crtp as crtp

class CrazyflieSwarmNode(Node):
  def __init__(self):
    super().__init__('crazyflie_swarm_node')
    self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    self.declare_parameter('package_path', '')
    root = self.get_parameter('package_path').get_parameter_value().string_value
    config = load_config(f'{root}/config/config.yaml', SwarmConfig)
    self.config = config

    self.get_logger().info(f'CrazyflieSwarmNode started with parameters:')
    for cf_config in self.config.crazyflies:
      self.get_logger().info(f'  - {cf_config.name}: {cf_config.uri}')
    
    #* CrazyflieSwarm
    crtp.init_drivers() 
    
    self.swarm: Dict[str, CrazyflieRobot] = {} 
    for crazyflie_config in self.config.crazyflies:
      uri = crazyflie_config.uri
      name = crazyflie_config.name
      multiranger = crazyflie_config.multiranger
      crazyflie_robot = CrazyflieRobot(uri, ro_cache='./ro_cache', rw_cache='./rw_cache', ros2_logger=self.get_logger(), multiranger=multiranger)  
      while not crazyflie_robot.initialize(): time.sleep(0.5)
      self.swarm[name] = crazyflie_robot
    
    
    #* Subscriptions
    self.led_subscribers: Dict[str, Subscription] = {}
    for name, _ in self.swarm.items():
      self.led_subscribers[name] = self.create_subscription(Float32, f'/{name}/led', lambda msg, name=name: self.led_callback(msg, name), 10)
    
            
    #* Publishers
    self.state_publishers: Dict[str, Publisher] = {}
    state_publisher_rate = self.config.state_publisher_rate
    for name, _ in self.swarm.items():
      publisher = self.create_publisher(CrazyflieState, f'/{name}/state', 10)
      self.state_publishers[name] = publisher
      self.create_timer(1/state_publisher_rate, lambda name=name, publisher=publisher: self.state_callback(name, publisher))
    
      
    #* Services
    self.take_off_service = self.create_service(TakeOff, '/take_off', self.take_off_service_callback)
    self.land_service = self.create_service(Land, '/land', self.land_service_callback)




  def led_callback(self, msg, name: str) -> None:    
    self.get_logger().info(f'Received message: {msg.data} for robot: {name}')
    try:
      self.swarm[name].set_led(int(msg.data))
    except Exception as e:
      self.get_logger().error(f'Error in led_callback: {e}')
  
  def velocity_callback(self, msg, name: str) -> None:
    self.get_logger().info(f'Received message: {msg.vx}, {msg.vy}, {msg.vz}, {msg.yawrate} for robot: {name}')
    try:
      self.swarm[name].set_velocity(msg.vx, msg.vy, msg.vz, msg.yawrate)   
    except Exception as e:
      self.get_logger().error(f'Error in velocity_callback: {e}')
            
  def state_callback(self, name, publisher) -> None: 
    try:
      state_msg = self.swarm[name].get_state()
      publisher.publish(state_msg)  
    except Exception as e:
      self.get_logger().error(f'Error in state_callback: {e}')
          
                
  def take_off_service_callback(self, request, response):
    self.get_logger().info(f'Take off')
    try:
      height = request.height
      velocity = request.velocity 
      for name, cf in self.swarm.items():
        cf.take_off(height, velocity)
      response.success = True

    except Exception as e:
      self.get_logger().error(f'Error in take_off_callback: {e}')
      response.success = False
      
    return response
  
  def land_service_callback(self, request, response):
    self.get_logger().info(f'Land')
    try:
      velocity = request.velocity
      for name, cf in self.swarm.items():
        cf.land(velocity)
      response.success = True

    except Exception as e:
      self.get_logger().error(f'Error in land_callback: {e}')
      response.success = False
    
    return response
                                              
def main(args=None):
  rclpy.init(args=args)
  crazyflie_swarm_node = CrazyflieSwarmNode()
  try:
    rclpy.spin(crazyflie_swarm_node)
    
  except KeyboardInterrupt:
    crazyflie_swarm_node.land_service_callback()
    pass
    
  except Exception as e:
    crazyflie_swarm_node.get_logger().error(f'Error in main loop: {e}')
    
  finally:
    crazyflie_swarm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()