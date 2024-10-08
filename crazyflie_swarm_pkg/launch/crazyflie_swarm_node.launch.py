from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
  ld = LaunchDescription()
  
  swarm=Node(
    package = 'crazyflie_swarm_pkg',
    name = 'crazyflie_dock_node',
    executable = 'crazyflie_dock_exec'
  )
      
  ld.add_action(swarm)
  
  return ld