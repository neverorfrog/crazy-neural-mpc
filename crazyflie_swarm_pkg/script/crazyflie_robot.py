import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import time
from typing import Dict
from threading import Event

from crazyflie_state import CrazyState
from utils import log

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils.multiranger import Multiranger

class CrazyflieRobot:
  def __init__(self, uri, ro_cache=None, rw_cache=None, ros2_logger=None, multiranger=False):        
    self.uri = uri
    self.cf = Crazyflie(ro_cache=ro_cache, rw_cache=rw_cache)
    self.scf = SyncCrazyflie(self.uri, cf=self.cf)      
    self.ros2_logger = ros2_logger
     
    self.default_height = 0.2
    self.default_velocity = 0.1
    
    # State
    self.state = CrazyState()
    self.estimators: Dict[str, SyncLogger] = {}
    
    # Connection
    self.__connection_timeout = 10 # seconds  
    self.__connection_opened = False
    
    # Flow deck
    self.__flow_deck_attached = False
    self.flow_deck_attached_event = Event()
    self.flow_deck_attached_event.clear()
    
    # Multiranger
    self.multiranger = multiranger
    self.__multiranger_attached = False
    self.multiranger_attached_event = Event()
    self.multiranger_attached_event.clear()
    self.multiranger_sensor = Multiranger(self.scf)
    
              
  #* Initialization
  def initialize(self):
    start_initialization = time.time()
    
    self.open_connection()
    self.scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=self.flow_deck_attached_callback)
    if self.multiranger: self.scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=self.multiranger_deck_attached_callback)
    
    log(f'Connecting to Crazyflie {self.uri} ...', self.ros2_logger)
    while not self.__connection_opened or \
          not self.__flow_deck_attached or \
          (not self.__multiranger_attached and self.multiranger):
        
      if time.time() - start_initialization > self.__connection_timeout:
        log(f'Initialization timeout for {self.uri}', self.ros2_logger)
        self.close_connection()
        return False
      time.sleep(0.1)
    
    log(f'Crazyflie {self.uri} connected.', self.ros2_logger)
          
    log(f'Resetting estimators of Crazyflie {self.uri} ...', self.ros2_logger)
    self.reset_estimator()
    log(f'Estimators of Crazyflie {self.uri} reset.', self.ros2_logger)
            
    log(f'Starting estimators of Crazyflie {self.uri} ...', self.ros2_logger)      
    self.setup_estimators()
    log(f'Estimators of Crazyflie {self.uri} started.', self.ros2_logger)

    log(f'Crazyflie {self.uri} initialized', self.ros2_logger)
    return True
        
  def destroy(self):
    self.multiranger_sensor.stop()
    self.multiranger_attached_event.clear()
    self.__multiranger_attached = False
    
    self.flow_deck_attached_event.clear()
    self.__flow_deck_attached = False
    
    self.destroy_estimators()
  
    self.close_connection()
    log(f'Crazyflie {self.uri} destroyed', self.ros2_logger)      
        
  # Flow deck management
  def flow_deck_attached_callback(self, _, value_str):
    if int(value_str):
      self.flow_deck_attached_event.set()
      self.__flow_deck_attached = True
      log(f'Flow deck attached to {self.uri}', self.ros2_logger)
    else:
      log(f'Flow deck is not attached to {self.uri}', self.ros2_logger)

  def multiranger_deck_attached_callback(self, _, value_str) -> None:
    if int(value_str):
      self.multiranger_attached_event.set()
      self.__multiranger_attached = True
      self.multiranger_sensor.start()
      log(f'Multiranger is attached to {self.uri}', self.ros2_logger)
    else:
      log(f'Multiranger is not attached to {self.uri}', self.ros2_logger)

  # Connection management
  def open_connection(self):
    if self.__connection_opened: raise Exception('Connection already opened')
    try:
      self.scf.open_link()
      self.__connection_opened = True 
    except Exception as e:
      self.close_connection()
      raise e
       
  def close_connection(self):    
    self.__connection_opened = False
    self.scf.close_link()
        
  #* Commands
  def take_off(self, absolute_height=None, velocity=None):
    if not self.__connection_opened: raise Exception('Connection not opened')
    if not self.__flow_deck_attached: raise Exception('Flow deck not attached')
    if self.multiranger and not self.__multiranger_attached: raise Exception('Multiranger not attached')

    if absolute_height is None: absolute_height = self.default_height
    if velocity is None: velocity = self.default_velocity
    self.cf.high_level_commander.takeoff(absolute_height, velocity)
        
  def land(self, absolute_height=None, velocity=None):     
    if absolute_height is None: absolute_height = self.default_height
    if velocity is None: velocity = self.default_velocity   
    self.cf.commander.send_notify_setpoint_stop()
    self.cf.high_level_commander.land(absolute_height, velocity)
    self.cf.high_level_commander.stop()
        
  def set_velocity(self, vx, vy, vz, yaw_rate):
    self.cf.commander.send_velocity_world_setpoint(vx, vy, vz, yaw_rate)   
     
  def hover(self):
    pass
       
                    
  #* Setters
  def set_led(self, intensity):
    self.cf.param.set_value('led.bitmask', intensity)
        
                            
  #* Getters    
  def get_state(self) -> CrazyState:
    
    pose_estimator_entry = self.estimators['pose'].next()
    velocity_estimator_entry = self.estimators['velocity'].next()
    
    self.state.x = pose_estimator_entry[1]['stateEstimate.x']
    self.state.y = pose_estimator_entry[1]['stateEstimate.y']
    self.state.z = pose_estimator_entry[1]['stateEstimate.z']
    
    self.state.roll = pose_estimator_entry[1]['stabilizer.roll']
    self.state.pitch = pose_estimator_entry[1]['stabilizer.pitch']
    self.state.yaw = pose_estimator_entry[1]['stabilizer.yaw']
    
    self.state.vx = velocity_estimator_entry[1]['stateEstimate.vx']
    self.state.vy = velocity_estimator_entry[1]['stateEstimate.vy']
    self.state.vz = velocity_estimator_entry[1]['stateEstimate.vz']
    
    self.state.roll_rate = velocity_estimator_entry[1]['stateEstimateZ.rateRoll']
    self.state.pitch_rate = velocity_estimator_entry[1]['stateEstimateZ.ratePitch']
    self.state.yaw_rate = velocity_estimator_entry[1]['stateEstimateZ.rateYaw']
    
    self.state.mr_front = self.multiranger_sensor.front
    self.state.mr_right = self.multiranger_sensor.right
    self.state.mr_back = self.multiranger_sensor.back
    self.state.mr_left = self.multiranger_sensor.left
    self.state.mr_up = self.multiranger_sensor.up
        
    return self.state
  
  
  def setup_estimators(self):
    pose_estimator = LogConfig(name="Pose", period_in_ms=10)
    pose_estimator.add_variable("stateEstimate.x", "float")
    pose_estimator.add_variable("stateEstimate.y", "float")
    pose_estimator.add_variable("stateEstimate.z", "float")
    pose_estimator.add_variable("stabilizer.roll", "float")
    pose_estimator.add_variable("stabilizer.pitch", "float")
    pose_estimator.add_variable("stabilizer.yaw", "float")
    pose_estimator_logger = SyncLogger(self.scf, pose_estimator)
    pose_estimator_logger.connect()
    self.estimators['pose'] = pose_estimator_logger
    
    velocity_estimator = LogConfig(name="velocity", period_in_ms=10)
    velocity_estimator.add_variable("stateEstimate.vx", "float")
    velocity_estimator.add_variable("stateEstimate.vy", "float")
    velocity_estimator.add_variable("stateEstimate.vz", "float")
    velocity_estimator.add_variable("stateEstimateZ.rateRoll", "float")
    velocity_estimator.add_variable("stateEstimateZ.ratePitch", "float")
    velocity_estimator.add_variable("stateEstimateZ.rateYaw", "float")
    velocity_estimator_logger = SyncLogger(self.scf, velocity_estimator)
    velocity_estimator_logger.connect()
    self.estimators['velocity'] = velocity_estimator_logger
    
  def destroy_estimators(self):
    for estimator in self.estimators.values():
      estimator.disconnect()
  
  #* Estimator Reset   
  def reset_estimator(self):
    self.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    self.cf.param.set_value('kalman.resetEstimation', '0')
    
    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_y_history = [1000] * 10
    var_x_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(self.scf, log_config) as logger:
      for log_entry in logger:
        data = log_entry[1]

        var_x_history.append(data['kalman.varPX'])
        var_x_history.pop(0)
        var_y_history.append(data['kalman.varPY'])
        var_y_history.pop(0)
        var_z_history.append(data['kalman.varPZ'])
        var_z_history.pop(0)

        min_x = min(var_x_history)
        max_x = max(var_x_history)
        min_y = min(var_y_history)
        max_y = max(var_y_history)
        min_z = min(var_z_history)
        max_z = max(var_z_history)
                
        if (max_x - min_x) < threshold and (max_y - min_y) < threshold and (max_z - min_z) < threshold:
          break
      