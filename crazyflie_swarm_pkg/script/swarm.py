import cflib.crtp as crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.motion_commander import MotionCommander
import time
from threading import Event

class CrazyflieRobot:
    def __init__(self, uri, ro_cache=None, rw_cache=None):        
        self.uri = uri
        self.cf = Crazyflie(ro_cache=ro_cache, rw_cache=rw_cache)
        self.scf = SyncCrazyflie(self.uri, cf=self.cf)      
        
        self.default_height = 0.2
        self.default_velocity = 0.1
        self.motion_commander = MotionCommander(self.scf, self.default_height)
          
        self.__timeout = 10 # seconds  
        self.__connection_opened = False
        self.__flow_deck_attached = False
        self.flow_deck_attached_event = Event()
        self.flow_deck_attached_event.clear()
        
              
    #* Initialization
    def initialize(self):
        start_initialization = time.time()
        
        self.open_connection()
        self.scf.cf.param.add_update_callback(group="deck", name="bcFlow2", cb=self.flow_deck_attached_callback)
        
        while not self.__connection_opened or \
              not self.__flow_deck_attached:
            
            if time.time() - start_initialization > self.__timeout:
                print(f'Initialization timeout for {self.uri}')
                self.close_connection()
                return False
            
            time.sleep(0.1)
        
        print(f'Crazyflie {self.uri} initialized')
        return True
        
    # Flow deck management
    def flow_deck_attached_callback(self, _, value_str):
        if int(value_str):
            self.flow_deck_attached_event.set()
            self.__flow_deck_attached = True
            print(f'Flow deck attached to {self.uri}')
        else:
            print(f'Flow deck is not attached to {self.uri}')

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
        
    #* Commands
    def take_off(self, absolute_height=None, velocity=None):
        if not self.__connection_opened: raise Exception('Connection not opened')
        if not self.__flow_deck_attached: raise Exception('Flow deck not attached')
        
        height = self.default_height if absolute_height is None else absolute_height
        velocity = self.default_velocity if velocity is None else velocity
        self.motion_commander.take_off(height, velocity)
        
    def land(self, velocity=None):        
        velocity = self.default_velocity if velocity is None else velocity
        self.motion_commander.land(velocity)
        
    def hover(self):
        self.motion_commander.stop()
                            
    #* Getters    
    def get_sync_crazyflie(self):
        return self.scf
    
    def get_estimated_position(self):
        log_config = LogConfig(name='State', period_in_ms=10)
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')
        position = {'x': 0, 'y': 0, 'z': 0}
        with SyncLogger(self.scf, log_config) as logger:
            for entry in logger:
                position['x'] = entry[1]['stateEstimate.x']
                position['y'] = entry[1]['stateEstimate.y']
                position['z'] = entry[1]['stateEstimate.z']
                break
        return position
    
    def get_estimated_euler_orientation(self):
        log_config = LogConfig(name='State', period_in_ms=10)
        log_config.add_variable('stateEstimate.roll', 'float')
        log_config.add_variable('stateEstimate.pitch', 'float')
        log_config.add_variable('stateEstimate.yaw', 'float')
        orientation = {'roll': 0, 'pitch': 0, 'yaw': 0}
        with SyncLogger(self.scf, log_config) as logger:
            for entry in logger:
                orientation['roll'] = entry[1]['stateEstimate.roll']
                orientation['pitch'] = entry[1]['stateEstimate.pitch']
                orientation['yaw'] = entry[1]['stateEstimate.yaw']
                break
        return orientation
    
    def get_estimated_quaternion_orientation(self):
        log_config = LogConfig(name='State', period_in_ms=10)
        log_config.add_variable('stateEstimate.qx', 'float')
        log_config.add_variable('stateEstimate.qy', 'float')
        log_config.add_variable('stateEstimate.qz', 'float')
        log_config.add_variable('stateEstimate.qw', 'float')
        orientation = {'qx': 0, 'qy': 0, 'qz': 0, 'qw': 0}
        with SyncLogger(self.scf, log_config) as logger:
            for entry in logger:
                orientation['qx'] = entry[1]['stateEstimate.qx']
                orientation['qy'] = entry[1]['stateEstimate.qy']
                orientation['qz'] = entry[1]['stateEstimate.qz']
                orientation['qw'] = entry[1]['stateEstimate.qw']
                break
        return orientation
    
    def get_estimated_linear_velocity(self):
        log_config = LogConfig(name='Velocity', period_in_ms=10)
        log_config.add_variable('stateEstimate.vx', 'float')
        log_config.add_variable('stateEstimate.vy', 'float')
        log_config.add_variable('stateEstimate.vz', 'float')
        linear_velocity = {'vx': 0, 'vy': 0, 'vz': 0}
        with SyncLogger(self.scf, log_config) as logger:
            for entry in logger:
                linear_velocity['vx'] = entry[1]['stateEstimate.vx']
                linear_velocity['vy'] = entry[1]['stateEstimate.vy']
                linear_velocity['vz'] = entry[1]['stateEstimate.vz']
                break
        return linear_velocity
    
    def get_estimated_acceleration(self):
        log_config = LogConfig(name='Acceleration', period_in_ms=10)   
        log_config.add_variable('stateEstimate.ax', 'float')
        log_config.add_variable('stateEstimate.ay', 'float')
        log_config.add_variable('stateEstimate.az', 'float')
        acceleration = {'ax': 0, 'ay': 0, 'az': 0}
        with SyncLogger(self.scf, log_config) as logger:
            for entry in logger:
                acceleration['ax'] = entry[1]['stateEstimate.ax']
                acceleration['ay'] = entry[1]['stateEstimate.ay']
                acceleration['az'] = entry[1]['stateEstimate.az']
                break
        return acceleration
    
        
    #* Setters
    def set_led(self, intensity):
        self.cf.param.set_value('led.bitmask', intensity)
    
    
    
class CrazyflieSwarm:
    def __init__(self, uris, ro_cache=None, rw_cache=None):
        crtp.init_drivers()
        self.uris = uris    
        self.__crazyflie_robots = {}
        
        self.__timeout = 10 # seconds
          
        for uri in uris:
            crazyflie_robot = CrazyflieRobot(uri, ro_cache=ro_cache, rw_cache=rw_cache)
            self.__crazyflie_robots[uri] = crazyflie_robot 
        
    #* Connection management
    def open_connections(self):
        start_connections = time.time()
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            while not crazyflie_robot.initialize():
                print(f'Waiting for {uri} to initialize ...')
                time.sleep(0.5)
                if time.time() - start_connections > self.__timeout:
                    print(f'Initialization timeout')
                    self.close_connections()
                    return False
        return True
    
    def close_connections(self):
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            crazyflie_robot.close_connection()
        
    #* Commands
    def take_off(self, dict):
        print("Taking off ...")
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            print(f" - taking off {uri}")   
            absolute_height = dict[uri]['absolute_height']
            velocity = dict[uri]['velocity']
            crazyflie_robot.take_off(absolute_height, velocity)
            
    def land(self, dict):
        print("Landing ...")
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            print(f" - landing {uri}")
            velocity = dict[uri]['velocity']
            crazyflie_robot.land(velocity)
            
    def hover(self):
        print("Hovering ...")
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            print(f" - hovering {uri}")
            crazyflie_robot.hover()
    
    #* Getters      
    def get_estimated_positions(self):  
        estimated_positions = {uri: {'x': 0, 'y': 0, 'z': 0} for uri in self.uris}
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            position = crazyflie_robot.get_estimated_position()
            estimated_positions[uri] = position
        return estimated_positions
    
    def get_estimated_euler_orientations(self):
        estimated_euler_orientations = {uri: {'roll': 0, 'pitch': 0, 'yaw': 0} for uri in self.uris}
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            orientation = crazyflie_robot.get_estimated_euler_orientation()
            estimated_euler_orientations[uri] = orientation
        return estimated_euler_orientations
    
    def get_estimated_quaternion_orientations(self):
        estimated_quaternion_orientations = {uri: {'qx': 0, 'qy': 0, 'qz': 0, 'qw': 0} for uri in self.uris}
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            orientation = crazyflie_robot.get_estimated_quaternion_orientation()
            estimated_quaternion_orientations[uri] = orientation
        return estimated_quaternion_orientations
    
    def get_estimated_linear_velocities(self):
        estimated_linear_velocities = {uri: {'vx': 0, 'vy': 0, 'vz': 0} for uri in self.uris}
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            velocity = crazyflie_robot.get_estimated_linear_velocity()
            estimated_linear_velocities[uri] = velocity
        return estimated_linear_velocities
    
    def get_estimated_accelerations(self):
        estimated_accelerations = {uri: {'ax': 0, 'ay': 0, 'az': 0} for uri in self.uris}
        for uri, crazyflie_robot in self.__crazyflie_robots.items():
            acceleration = crazyflie_robot.get_estimated_acceleration()
            estimated_accelerations[uri] = acceleration
        return estimated_accelerations
    
    #* Setters
    def set_leds(self, dict):
        for uri, intensity in dict.items():
            self.__crazyflie_robots[uri].set_led(intensity)