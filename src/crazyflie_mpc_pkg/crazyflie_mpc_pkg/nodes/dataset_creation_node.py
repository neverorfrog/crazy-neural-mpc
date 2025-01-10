import os
import rclpy
import rclpy.logging
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import numpy as np
from ament_index_python.packages import get_package_share_directory

from crazyflie_swarm_interfaces.msg import CrazyflieState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

logger = rclpy.logging.get_logger("dataset_creation_node")
root = get_package_share_directory("crazyflie_mpc_pkg")

class RosbagExtractor:
    def __init__(self, bag_path):
        storage_options = StorageOptions(
            uri=bag_path,
            storage_id='sqlite3'
        )
        converter_options = ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        
        self.reader = SequentialReader()
        self.reader.open(storage_options, converter_options)
        
    def extract_messages(self, topic_name: str, topic_type):
        data = []
        timestamps = []
        
        while self.reader.has_next():
            topic, msg, t = self.reader.read_next()
            if topic == topic_name:
                msg_deserialized = deserialize_message(msg, topic_type)
                data.append(msg_deserialized)
                timestamps.append(t)
                
        self.reader.seek(0)
        return np.array(timestamps), np.array(data)

def main():
    # Initialize ROS
    rclpy.init()
    
    trajectories = ["horizontal_circle", "vertical_circle", "lemniscate"]
    state_traj = []
    command_traj = []
    time_traj = []
    
    for trajectory in trajectories:
        logger.info(f"Extracting dataset for {trajectory} trajectory.")
        bag_path = os.path.join(root, f"bags/{trajectory}/bag.db3")
        extractor = RosbagExtractor(bag_path)
    
        # Extract data from specific topics
        t_commands, commands = extractor.extract_messages("/cf_1/cmd_attitude_setpoint", Twist)
        t_states, states = extractor.extract_messages("/cf_1/state", CrazyflieState)
        t_references, references = extractor.extract_messages("/cf_1/reference", Path)
    
        logger.info(f"Extracted {len(commands)} commands.")
        logger.info(f"Extracted {len(states)} states.")
        logger.info(f"Extracted {len(references)} references.")
    
        start_time = min(t_commands[0], t_states[0], t_references[0])
        end_time = max(t_commands[-1], t_states[-1], t_references[-1])
        elapsed = end_time - start_time
    
        delta = 0.01  # 10 ms
        step_size = int(delta * 1e9)  # Convert to nanoseconds
        current_time = start_time
        while current_time <= end_time:
            # Find closest data points to current time
            cmd_idx = np.argmin(np.abs(t_commands - current_time))
            state_idx = np.argmin(np.abs(t_states - current_time))
            ref_idx = np.argmin(np.abs(t_references - current_time))
            
            # Process data at these indices here
            state: CrazyflieState = states[state_idx]
            state_vector = np.hstack([state.position, state.linear_velocity, state.euler_orientation])
            state_traj.append(state_vector)
            
            command: Twist = commands[cmd_idx]
            command_vector = np.array([command.linear.x, command.linear.y, command.linear.z, command.angular.x, command.angular.y, command.angular.z])
            command_traj.append(command_vector)
            
            time_traj.append((current_time - start_time) / 1e9)
            
            current_time += step_size
    
        logger.info(f"Start time: {start_time}")
        logger.info(f"End time: {end_time}")
        logger.info(f"Elapsed time: {elapsed / 1e9} seconds")
    
    state_traj = np.array(state_traj)
    command_traj = np.array(command_traj)
    time_traj = np.array(time_traj)
    
    dataset = np.hstack([state_traj, command_traj, time_traj.reshape(-1, 1)])
    logger.info(f"Dataset shape: {dataset.shape}") # [num_samples, 16] -> [position, linear_velocity, euler_orientation, linear_velocity_command, angular_velocity_command, time]
    np.save(os.path.join(root, "bags/dataset.npy"), dataset)
    
    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()