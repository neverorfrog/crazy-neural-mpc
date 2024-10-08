import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from typing import Dict
from config import SwarmConfig, load_config, get_package_root
import cflib.crtp as crtp
from crazyflie_robot import CrazyflieRobot
import time

from crazyflie_swarm_interfaces.msg import CrazyflieState

class CrazyflieSwarm():
  def __init__(self, ros2_logger=None):
    root = get_package_root()     
    config = load_config(f'{root}/config/config.yaml', SwarmConfig)
    self.config = config
    
    crtp.init_drivers() 
    self._crazyflies: Dict[str, CrazyflieRobot] = {} 
    
    for crazyflie_config in self.config.crazyflies:
      uri = crazyflie_config.uri
      name = crazyflie_config.name
      crazyflie_robot = CrazyflieRobot(uri, ro_cache='./ro_cache', rw_cache='./rw_cache')
  
      while not crazyflie_robot.initialize():
          ros2_logger.info(f'Connecting to Crazyflie {name} ...')
          time.sleep(0.5)
      ros2_logger.info(f'Connected to Crazyflie {name}')
      
      ros2_logger.info(f'Resetting estimators of Crazyflie {name} ...')
      crazyflie_robot.reset_estimator()
      ros2_logger.info(f'Estimators of Crazyflie {name} reset.')
      
      self._crazyflies[name] = crazyflie_robot
    
    self._iterator = iter(self._crazyflies.items())
      
        
  def set_led(self, name, led_value):
    self._crazyflies[name].set_led(led_value)
    
  def get_state(self, name) -> CrazyflieState:
    cf = self._crazyflies[name]
  
    state_msg = CrazyflieState()
    state_msg.uri = cf.get_uri()
    
    state_msg.position[0] = cf.state.x
    state_msg.position[1] = cf.state.y
    state_msg.position[2] = cf.state.z
    
    state_msg.euler_orientation[0] = cf.state.roll
    state_msg.euler_orientation[1] = cf.state.pitch
    state_msg.euler_orientation[2] = cf.state.yaw
    
    state_msg.quaternion_orientation[0] = cf.state.qx
    state_msg.quaternion_orientation[1] = cf.state.qy
    state_msg.quaternion_orientation[2] = cf.state.qz
    state_msg.quaternion_orientation[3] = cf.state.qw
    
    state_msg.velocity[0] = cf.state.vx
    state_msg.velocity[1] = cf.state.vy
    state_msg.velocity[2] = cf.state.vz
        
  def __getitem__(self, name):
    return self._crazyflies[name]
        
  def __iter__(self):
    self._iterator = iter(self._crazyflies.items())
    return self
  
  def __next__(self):
    return next(self._iterator)

  def __len__(self):
    return len(self._crazyflies)