from launch import LaunchDescription
from launch_ros.actions import Node
    
def generate_launch_description():
  ld = LaunchDescription()
        
  dock=Node(
    package = 'crazyflie_swarm_pkg',
    name = 'crazyflie_dock_node',
    executable = 'crazyflie_dock_exec'
  )
  
  ld.add_action(dock)
  
  return ld