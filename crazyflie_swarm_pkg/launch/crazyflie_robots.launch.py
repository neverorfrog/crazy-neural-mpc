import os
import time
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

robots_description = {
  'crazyflie_robot_1': 'radio://0/80/2M/E7E7E7E7E7',
  'crazyflie_robot_2': 'radio://0/80/2M/E7E7E7E7E8',
}

def generate_launch_description():

  ld = LaunchDescription()
  
  config = os.path.join(
    get_package_share_directory('crazyflie_swarm_pkg'),
    'config',
    'crazyflie_robot_node.yaml'
  )
        
  for name, uri in robots_description.items():
    crazyflie_robot_node = Node(
      package = 'crazyflie_swarm_pkg',
      name = 'crazyflie_robot_node',
      executable = 'crazyflie_robot_exec',
      parameters = [
        {'name': name},
        {'uri': uri},
        config,
      ]
    )
    
    ld.add_action(crazyflie_robot_node)    
      
  return ld