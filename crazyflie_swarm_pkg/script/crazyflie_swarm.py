import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from typing import Dict
from config import SwarmConfig, load_config, get_package_root
from utils import log
from crazyflie_robot import CrazyflieRobot

import cflib.crtp as crtp
import time

from crazyflie_swarm_interfaces.msg import CrazyflieState

class CrazyflieSwarm():
  def __init__(self, ros2_logger=None):
    self.ros2_logger = ros2_logger
    
    root = get_package_root()     
    config = load_config(f'{root}/config/config.yaml', SwarmConfig)
    self.config = config
    
    crtp.init_drivers() 
    self._crazyflies: Dict[str, CrazyflieRobot] = {} 
    
    for crazyflie_config in self.config.crazyflies:
      uri = crazyflie_config.uri
      name = crazyflie_config.name
      multiranger = crazyflie_config.multiranger
      crazyflie_robot = CrazyflieRobot(uri, ro_cache='./ro_cache', rw_cache='./rw_cache', ros2_logger=self.ros2_logger, multiranger=multiranger)
  
      while not crazyflie_robot.initialize():
        log(f'Connecting to Crazyflie {name} ...', self.ros2_logger)
        time.sleep(0.5)
      log(f'Crazyflie {name} connected.', self.ros2_logger)

      log(f'Resetting estimators of Crazyflie {name} ...', self.ros2_logger)          
      # crazyflie_robot.reset_estimator()
      log(f'Estimators of Crazyflie {name} reset.', self.ros2_logger)
            
      self._crazyflies[name] = crazyflie_robot
    
    self._iterator = iter(self._crazyflies.items())
      
  #* Setters
  def set_led(self, name, led_value):
    self._crazyflies[name].set_led(led_value)
    
  def set_velocity(self, name, vx, vy, vz, yawrate):
    self._crazyflies[name].set_velocity(vx, vy, vz, yawrate)
  
  #* Getters
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
    
    state_msg.multiranger[0] = cf.multiranger_sensor.front
    state_msg.multiranger[1] = cf.multiranger_sensor.back
    state_msg.multiranger[2] = cf.multiranger_sensor.left
    state_msg.multiranger[3] = cf.multiranger_sensor.right
    state_msg.multiranger[4] = cf.multiranger_sensor.up
    
    return state_msg
  
  def get_multiranger_data(self, name):
    return self._crazyflies[name].get_multiranger_data()
  
  #* Properties
  def __getitem__(self, name):
    return self._crazyflies[name]
        
  def __iter__(self):
    self._iterator = iter(self._crazyflies.items())
    return self
  
  def __next__(self):
    return next(self._iterator)

  def __len__(self):
    return len(self._crazyflies)