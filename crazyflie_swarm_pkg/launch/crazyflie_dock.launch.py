import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
    
def generate_launch_description():
  ld = LaunchDescription()
  
  config = os.path.join(
    get_package_share_directory('crazyflie_swarm_pkg'),
    'config',
    'crazyflie_dock_node.yaml'
  )
      
  node=Node(
    package = 'crazyflie_swarm_pkg',
    name = 'crazyflie_dock_node',
    executable = 'crazyflie_dock_exec',
    parameters = [config]
  )
  
  ld.add_action(node)
  
  return ld