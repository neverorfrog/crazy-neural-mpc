import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():  
  ld = LaunchDescription()
  
  config = os.path.join(
    get_package_share_directory('crazyflie_swarm_pkg'),
    'config',
    'crazyflie_robot_node.yaml'
  )
      
  crazyflie_robot_1 = Node(
    package = 'crazyflie_swarm_pkg',
    name = 'crazyflie_robot_node',
    executable = 'crazyflie_robot_exec',
    parameters = [
      {'name': 'crazyflie_robot_1'},
      {'uri': 'radio://0/80/2M/E7E7E7E7E7'},
      config,
    ]
  )
  
  ld.add_action(crazyflie_robot_1)
  
  return ld