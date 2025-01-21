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
    
    state_traj = []
    command_traj = []
    time_traj = []
    label_traj = []
    
    for folder in os.listdir(os.path.join(root, "bags/gazebo")):
        bag_path = os.path.join(root, f"bags/gazebo/{folder}")
        
        for f in os.listdir(bag_path):
            if f.startswith('rosbag2'):
                file_name = f
                break
                    
        file_path = os.path.join(bag_path, file_name)
        extractor = RosbagExtractor(file_path)
    
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
    
        transition_delta = 0.024  # 24 ms 
        subtransition_delta = 0.006 # 6 ms
        substep_size = int(subtransition_delta * 1e9)
        current_time = start_time
        while current_time <= end_time:
            
            transition_start_time = current_time
            
            cmd_idx = np.argmin(np.abs(t_commands - current_time))
            command: Twist = commands[cmd_idx]
            command_vector = np.array([command.angular.x, command.angular.y, command.angular.z, command.linear.z])
            
            state_start_idx = np.argmin(np.abs(t_states - current_time))
            state_idx = state_start_idx
            for _ in range(int(transition_delta / subtransition_delta)):
                state: CrazyflieState = states[state_start_idx]
                state_vector = np.hstack([state.position, state.linear_velocity, state.euler_orientation])
                state_traj.append(state_vector)
                
                label: CrazyflieState = states[state_idx]
                label_vector = np.hstack([label.position, label.linear_velocity, label.euler_orientation])
                label_traj.append(label_vector)
                
                time_traj.append((current_time - transition_start_time) / 1e9)
                command_traj.append(command_vector)
                
                current_time = current_time + substep_size
                state_idx = np.argmin(np.abs(t_states - current_time))
    
        logger.info(f"Start time: {start_time}")
        logger.info(f"End time: {end_time}")
        logger.info(f"Elapsed time: {elapsed / 1e9} seconds")
    
    state_traj = np.array(state_traj)
    command_traj = np.array(command_traj)
    time_traj = np.array(time_traj)
    label_traj = np.array(label_traj)
    
    np.set_printoptions(precision=3, suppress=True)
    
    logger.info(f"State trajectory shape: {state_traj.shape}")
    logger.info(f"Command trajectory shape: {command_traj.shape}")
    
    start_idx = 1000
    end_idx = start_idx + (int(transition_delta / subtransition_delta))
    logger.info(f"TEN STATE TRAJECTORY: \n {state_traj[start_idx:end_idx, :]}")
    logger.info(f"TEN COMMAND TRAJECTORY: \n {command_traj[start_idx:end_idx, :]}")
    logger.info(f"TEN TIME TRAJECTORY: \n {time_traj[start_idx:end_idx]}")
    
    logger.info(f"TEN LABEL TRAJECTORY: \n {label_traj[start_idx:end_idx, :]}")
    
    X = np.hstack([state_traj, command_traj, time_traj.reshape(-1, 1)])
    logger.info(f"TEN DATASET SAMPLE TRAJECTORY: \n {X[start_idx:end_idx, :]}")
    
    Y = label_traj
    logger.info(f"Dataset shape: {X.shape}") # [num_samples, 16] -> [position, linear_velocity, euler_orientation, command, time]
    np.save(os.path.join(root, "bags/X.npy"), X)
    np.save(os.path.join(root, "bags/Y.npy"), Y)
    
    # Clean up
    rclpy.shutdown()

if __name__ == '__main__':
    main()