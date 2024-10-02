import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory
from cflib.crazyflie import syncCrazyflie
from script.swarm import Swarm

class CrazyflieSwarm():
    
    def __init__(self, uris):
        
        self.uris = uris 
              
        cflib.crtp.init_drivers()
        self.swarm = Swarm(self.uris, factory=CachedCfFactory(rw_cache='./cache'))
        # self.swarm.reset_estimators()
                
    def connect(self):
        self.swarm.open_links()    
        
    def disconnect(self):
        self.swarm.close_links()
        
    def get_estimated_positions(self):
        return self.swarm.get_estimated_positions()
    
    def get_estimated_euler_orientations(self):
        return self.swarm.get_estimated_euler_orientations()
    
    def get_estimated_quaternions_orientations(self):
        return self.swarm.get_estimated_quaternions_orientations()
        
    def command_leds(self, sequence):
        def run_sequence_led(scf: syncCrazyflie, sequence):
            for value in sequence:
                scf.cf.param.set_value('led.bitmask', value)
            
        self.swarm.parallel_safe(run_sequence_led, args_dict=sequence)
            