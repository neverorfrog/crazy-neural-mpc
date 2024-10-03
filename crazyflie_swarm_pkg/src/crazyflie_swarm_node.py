import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

from crazyflie_swarm_interfaces.msg import CrazyflieState
from crazyflie_swarm_interfaces.srv import TakeOff, Land

import cflib.crtp as crtp
from script.swarm import CrazyflieRobot

class CrazyflieSwarmNode(Node):
  def __init__(self):
    super().__init__('crazyflie_swarm_node')
    self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

    #* Parameters
    self.declare_parameter('crazyflie_robots.robot_1.name', 'crazyflie_robot_1')
    self.declare_parameter('crazyflie_robots.robot_1.uri', 'radio://0/80/2M/E7E7E7E7E7')
    self.declare_parameter('crazyflie_robots.robot_1.state_publisher_rate', 10.0)
    
    self.declare_parameter('crazyflie_robots.robot_2.name', 'crazyflie_robot_2')
    self.declare_parameter('crazyflie_robots.robot_2.uri', 'radio://0/80/2M/E7E7E7E7E8')
    self.declare_parameter('crazyflie_robots.robot_2.state_publisher_rate', 10.0)

    self.crazyflie_params = {
      self.get_parameter('crazyflie_robots.robot_1.name').get_parameter_value().string_value : 
      {
        'uri': self.get_parameter('crazyflie_robots.robot_1.uri').get_parameter_value().string_value,
        'state_publisher_rate': self.get_parameter('crazyflie_robots.robot_1.state_publisher_rate').get_parameter_value().double_value,
      },
      self.get_parameter('crazyflie_robots.robot_2.name').get_parameter_value().string_value : 
      {
        'uri': self.get_parameter('crazyflie_robots.robot_2.uri').get_parameter_value().string_value,
        'state_publisher_rate': self.get_parameter('crazyflie_robots.robot_2.state_publisher_rate').get_parameter_value().double_value,
      },
    }
        
    self.get_logger().info(f'CrazyflieDockNode started with parameters:')
    self.get_logger().info(f'- crazyflie_params: ')
    for name, params in self.crazyflie_params.items():
      self.get_logger().info(f'  - {name}: {params}')
      
    #* Subscriptions
    self.led_subscribers = {}
    for name, _ in self.crazyflie_params.items():
      self.led_subscribers[name] = self.create_subscription(Float32, 
                                                            f'/{name}/led', 
                                                            lambda msg, name=name: self.led_callback(msg, name), 
                                                            10)
              
    #* Publishers
    self.state_publishers = {}
    for name, params in self.crazyflie_params.items():
      state_publisher_rate = params['state_publisher_rate']
      publisher = self.create_publisher(CrazyflieState, f'/{name}/state', 10)
      self.state_publishers[name] = publisher
      self.create_timer(1/state_publisher_rate, lambda name=name, publisher=publisher: self.state_callback(name, publisher))
      
    #* Services
    self.take_off_service = self.create_service(TakeOff, '/take_off', self.take_off_service_callback)
    self.land_service = self.create_service(Land, '/land', self.land_service_callback)
    

    #* CrazyflieSwarm
    crtp.init_drivers()
    self.crazyflie_swarm = {}        
    for name, params in self.crazyflie_params.items():
      uri = params['uri']
      crazyflie_robot = CrazyflieRobot(uri, ro_cache='./ro_cache', rw_cache='./rw_cache')
      while not crazyflie_robot.initialize():
        time.sleep(0.5)
      self.crazyflie_swarm[name] = crazyflie_robot
    
      self.get_logger().info(f'Resetting estimators for robot {uri} ...')
      crazyflie_robot.reset_estimator() 
      self.get_logger().info(f'... done!')
      
  
  def led_callback(self, msg, name):    
    self.get_logger().info(f'Received message: {msg.data} for robot: {name}')
    self.crazyflie_swarm[name].set_led(int(msg.data))
            
  def state_callback(self, name, publisher):        
    try:
      crazyflie_robot = self.crazyflie_swarm[name]
      
      position = crazyflie_robot.get_estimated_position()
      euler_orientation = crazyflie_robot.get_estimated_euler_orientation()
      quaternion_orientation = crazyflie_robot.get_estimated_quaternion_orientation()
                  
      state_msg = CrazyflieState()
      state_msg.uri = crazyflie_robot.get_uri()
      state_msg.position[0] = position['x']
      state_msg.position[1] = position['y']
      state_msg.position[2] = position['z']
      state_msg.euler_orientation[0] = euler_orientation['roll']
      state_msg.euler_orientation[1] = euler_orientation['pitch']
      state_msg.euler_orientation[2] = euler_orientation['yaw']
      state_msg.quaternion_orientation[0] = quaternion_orientation['qx']
      state_msg.quaternion_orientation[1] = quaternion_orientation['qy']
      state_msg.quaternion_orientation[2] = quaternion_orientation['qz']
      state_msg.quaternion_orientation[3] = quaternion_orientation['qw']
      
      publisher.publish(state_msg)
        
    except Exception as e:
      self.get_logger().error(f'Error in poses_callback: {e}')
               
                
  def take_off_service_callback(self, request, response):
    self.get_logger().info(f'Take off')
    try:
      height = request.height
      velocity = request.velocity 
      
      for name, crazyflie_robot in self.crazyflie_swarm.items():
        crazyflie_robot.take_off(height, velocity)
        
      response.success = True

    except Exception as e:
      self.get_logger().error(f'Error in take_off_callback: {e}')
      response.success = False
      
    return response
  
  
  def land_service_callback(self, request, response):
    self.get_logger().info(f'Land')
    try:
      velocity = request.velocity

      for name, crazyflie_robot in self.crazyflie_swarm.items():
        crazyflie_robot.land(velocity)

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