from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

    
def generate_launch_description():
  ld = LaunchDescription()
  
  path = get_package_share_directory('crazyflie_swarm_pkg')
        
  dock=Node(
    package = 'crazyflie_swarm_pkg',
    name = 'crazyflie_dock_node',
    executable = 'crazyflie_dock_exec',
    parameters=[{"package_path": path}],
  )
  
  ld.add_action(dock)
  
  return ld