from typing import Dict
import rclpy
from rclpy.node import Node, Subscription, Publisher
from std_msgs.msg import Float32

from crazyflie_swarm_interfaces.msg import CrazyflieState
from crazyflie_swarm_interfaces.srv import TakeOff, Land

from script.crazyflie_swarm import CrazyflieSwarm

class CrazyflieSwarmNode(Node):
  def __init__(self):
    super().__init__('crazyflie_swarm_node')
    self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
    
    #* CrazyflieSwarm (with config inside)
    self.swarm = CrazyflieSwarm()

    self.get_logger().info(f'CrazyflieSwarmNode started with parameters:')
    for cf_config in self.swarm.config.crazyflies:
      self.get_logger().info(f'  - {cf_config.name}: {cf_config.uri}')
      
    #* Subscriptions
    self.led_subscribers: Dict[str, Subscription] = {}
    for name, _ in self.swarm:
      self.led_subscribers[name] = self.create_subscription(Float32, 
                                                            f'/{name}/led', 
                                                            lambda msg, name=name: self.led_callback(msg, name), 
                                                            10)
              
    #* Publishers
    self.state_publishers: Dict[str, Publisher] = {}
    state_publisher_rate = self.swarm.config.state_publisher_rate
    for name, _ in self.swarm:
      publisher = self.create_publisher(CrazyflieState, f'/{name}/state', 10)
      self.state_publishers[name] = publisher
      self.create_timer(1/state_publisher_rate, lambda name=name, publisher=publisher: self.state_callback(name, publisher))
      
    #* Services
    self.take_off_service = self.create_service(TakeOff, '/take_off', self.take_off_service_callback)
    self.land_service = self.create_service(Land, '/land', self.land_service_callback)

  def led_callback(self, msg, name: str) -> None:    
    self.get_logger().info(f'Received message: {msg.data} for robot: {name}')
    self.swarm.set_led(name, msg.data)
            
  def state_callback(self, name: str, publisher: Publisher) -> None:        
    try:
      state_msg = self.swarm.get_state(name)
      publisher.publish(state_msg)
    except Exception as e:
      self.get_logger().error(f'Error in poses_callback: {e}')
                
  def take_off_service_callback(self, request, response):
    self.get_logger().info(f'Take off')
    try:
      height = request.height
      velocity = request.velocity 
      
      for name, cf in self.swarm:
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

      for name, cf in self.swarm:
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