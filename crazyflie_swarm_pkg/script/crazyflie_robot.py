import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import time
from threading import Event

from crazyflie_state import CrazyState
from utils import log

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils.multiranger import Multiranger

class CrazyflieRobot:
  def __init__(self, uri, ro_cache=None, rw_cache=None, ros2_logger=None):        
    self.uri = uri
    self.cf = Crazyflie(ro_cache=ro_cache, rw_cache=rw_cache)
    self.scf = SyncCrazyflie(self.uri, cf=self.cf)      
    self.ros2_logger = ros2_logger
    
    self.default_height = 0.2
    self.default_velocity = 0.1
    
    self.state = CrazyState()
    self.estimators: list[LogConfig] = []
      
    self.__timeout = 10 # seconds  
    self.__connection_opened = False
    
    self.__flow_deck_attached = False
    self.flow_deck_attached_event = Event()
    self.flow_deck_attached_event.clear()
    
    self.multiranger_attached = False
    self.multiranger_attached_event = Event()
    self.multiranger_attached_event.clear()
    self.multiranger = Multiranger(self.scf)
    
              
  #* Initialization
  def initialize(self):
    start_initialization = time.time()
    
    self.open_connection()
    self.scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=self.flow_deck_attached_callback)
    self.scf.cf.param.add_update_callback(group="deck", name="bcMultiranger", cb=self.multiranger_deck_attached_callback)
    
    while not self.__connection_opened or \
          not self.__flow_deck_attached or \
          not self.multiranger_attached:
        
      if time.time() - start_initialization > self.__timeout:
        log(f'Initialization timeout for {self.uri}', self.ros2_logger)
        self.close_connection()
        return False
    
      time.sleep(0.1)
    
    self.setup_estimators()
    for estimator in self.estimators:
      estimator.start()
        
    log(f'Crazyflie {self.uri} initialized', self.ros2_logger)
    return True
        
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
      self.multiranger_attached = True
      self.multiranger.start()
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
    self.scf.close_link()
    self.__connection_opened = False
    self.multiranger.stop()
        
  #* Commands
  def take_off(self, absolute_height=None, velocity=None):
    if not self.__connection_opened: raise Exception('Connection not opened')
    if not self.__flow_deck_attached: raise Exception('Flow deck not attached')

    if absolute_height is None: absolute_height = self.default_height
    if velocity is None: velocity = self.default_velocity
    self.cf.high_level_commander.takeoff(absolute_height, velocity)
        
  def land(self, absolute_height=None, velocity=None):     
    if absolute_height is None: absolute_height = self.default_height
    if velocity is None: velocity = self.default_velocity   
    self.cf.high_level_commander.land(absolute_height, velocity)
    self.cf.high_level_commander.stop()
        
  def hover(self):
    pass
                    
  #* Setters
  def set_led(self, intensity):
    self.cf.param.set_value('led.bitmask', intensity)
                            
  #* Getters    
  def get_uri(self):
    return self.uri
  
  #* Estimator Setup 
  def setup_estimators(self) -> None:
    def get_position_callback(timestamp, data, logconf) -> None:
      self.state.x = data["stateEstimate.x"]
      self.state.y = data["stateEstimate.y"]
      self.state.z = data["stateEstimate.z"]

    def get_euler_callback(timestamp, data, logconf) -> None:
      self.state.roll = data["stabilizer.roll"]
      self.state.pitch = data["stabilizer.pitch"]
      self.state.yaw = data["stabilizer.yaw"]
          
    def get_quaternion_callback(timestamp, data, logconf) -> None:
      self.state.qx = data["stateEstimate.qx"]
      self.state.qy = data["stateEstimate.qy"]
      self.state.qz = data["stateEstimate.qz"]
      self.state.qw = data["stateEstimate.qw"]

    def get_linear_velocity_callback(timestamp, data, logconf) -> None:
      self.state.vx = data["stateEstimate.vx"]
      self.state.vy = data["stateEstimate.vy"]
      self.state.vz = data["stateEstimate.vz"]

    def get_angular_velocity_callback(timestamp, data, logconf) -> None:
      self.state.roll_rate = data["stateEstimateZ.rateRoll"]
      self.state.pitch_rate = data["stateEstimateZ.ratePitch"]
      self.state.yaw_rate = data["stateEstimateZ.rateYaw"]
          
    def get_acceleration_callback(timestamp, data, logconf) -> None:
      self.state.ax = data["stateEstimate.ax"]
      self.state.ay = data["stateEstimate.ay"]
      self.state.az = data["stateEstimate.az"]
            
    self.estimators = []

    # Position
    position_estimator = LogConfig(name="State", period_in_ms=10)
    position_estimator.add_variable("stateEstimate.x", "float")
    position_estimator.add_variable("stateEstimate.y", "float")
    position_estimator.add_variable("stateEstimate.z", "float")
    self.estimators.append(position_estimator)
    self.cf.log.add_config(position_estimator)
    position_estimator.data_received_cb.add_callback(get_position_callback)

    # Euler
    euler_estimator = LogConfig(name="euler", period_in_ms=10)
    euler_estimator.add_variable("stabilizer.roll", "float")
    euler_estimator.add_variable("stabilizer.pitch", "float")
    euler_estimator.add_variable("stabilizer.yaw", "float")
    self.estimators.append(euler_estimator)
    self.cf.log.add_config(euler_estimator)
    euler_estimator.data_received_cb.add_callback(get_euler_callback)
      
    # Quaternion
    quaternion_estimator = LogConfig(name="Quaternion", period_in_ms=10)
    quaternion_estimator.add_variable("stateEstimate.qx", "float")
    quaternion_estimator.add_variable("stateEstimate.qy", "float")
    quaternion_estimator.add_variable("stateEstimate.qz", "float")
    quaternion_estimator.add_variable("stateEstimate.qw", "float")
    self.estimators.append(quaternion_estimator)
    self.cf.log.add_config(quaternion_estimator)
    quaternion_estimator.data_received_cb.add_callback(get_quaternion_callback)

    # Linear Velocity
    linear_velocity_estimator = LogConfig(name="LinearVelocity", period_in_ms=10)
    linear_velocity_estimator.add_variable("stateEstimate.vx", "float")
    linear_velocity_estimator.add_variable("stateEstimate.vy", "float")
    linear_velocity_estimator.add_variable("stateEstimate.vz", "float")
    self.estimators.append(linear_velocity_estimator)
    self.cf.log.add_config(linear_velocity_estimator)
    linear_velocity_estimator.data_received_cb.add_callback(get_linear_velocity_callback)

    # Angular Velocity
    angular_velocity_estimator = LogConfig(name="AngularVelocity", period_in_ms=10)
    angular_velocity_estimator.add_variable("stateEstimateZ.rateRoll", "float")
    angular_velocity_estimator.add_variable("stateEstimateZ.ratePitch", "float")
    angular_velocity_estimator.add_variable("stateEstimateZ.rateYaw", "float")
    self.estimators.append(angular_velocity_estimator)
    self.cf.log.add_config(angular_velocity_estimator)
    angular_velocity_estimator.data_received_cb.add_callback(get_angular_velocity_callback)
      
    # Acceleration
    acceleration_estimator = LogConfig(name="Acceleration", period_in_ms=10)
    acceleration_estimator.add_variable("stateEstimate.ax", "float")
    acceleration_estimator.add_variable("stateEstimate.ay", "float")
    acceleration_estimator.add_variable("stateEstimate.az", "float")
    self.estimators.append(acceleration_estimator)
    self.cf.log.add_config(acceleration_estimator)
    acceleration_estimator.data_received_cb.add_callback(get_acceleration_callback)
    
      
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
      