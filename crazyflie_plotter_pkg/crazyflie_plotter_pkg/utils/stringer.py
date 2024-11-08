# Generated with ChatGPT
from ament_index_python.packages import get_package_share_directory
import numpy as np

import yaml
from omegaconf import OmegaConf

import os

root = get_package_share_directory("crazyflie_plotter_pkg")
file_path = os.path.join(root, "config", "msg_config.yaml")

with open(file_path, 'r') as file:
    config = OmegaConf.create(yaml.safe_load(file))

def string_pose(name, x, y, z, roll, pitch, yaw):
    return f'{name}_{config.scope_pose}:{x};{y};{z};{roll};{pitch};{yaw}'

def get_pose(string):
    x, y, z, roll, pitch, yaw = string.split(';')

    return [float(x), float(y), float(z), float(roll), float(pitch), float(yaw)]

def string_yaw_rate(name, yaw_rate):
    return f'{name}_{config.scope_yaw_rate}:{yaw_rate}'

def get_yaw_rate(string):
    return float(string)

def string_option(name, opt):
    return f'{name}_{config.scope_opt}:{opt}'

def get_option(string):
    return int(string)

def string_dists(name, d1, d2, d3, d4):
    return f'{name}_{config.scope_dists}:{d1};{d2};{d3};{d4}'

def get_dists(string):
    d1, d2, d3, d4 = string.split(';')

    return [float(d1), float(d2), float(d3), float(d4)]

def string_controls(name, vx, vy, yaw_rate, z_distance):
    return f'{name}_{config.scope_controls}:{vx};{vy};{yaw_rate};{z_distance}'

def get_controls(string):
    vx, vy, yaw_rate, z_distance = string.split(';')

    return [float(vx), float(vy), float(yaw_rate), float(z_distance)]

def string_velocity(name, vx, vy):
    return f'{name}_{config.scope_velocity}:{vx};{vy}'

def string_velocity_noname(vx, vy):
    return f'{vx};{vy}'

def get_velocity(string):
    vx, vy = string.split(';')

    return [float(vx), float(vy)]

def string_forces(matrix):
    """
    Convert a 3x3 numpy matrix to a string representation.

    Parameters:
        matrix (np.ndarray): A 3x3 numpy array.

    Returns:
        str: A string representation of the matrix.
    """
    if matrix.shape != (3, 3):
        raise ValueError("Input must be a 3x3 matrix.")
    
    # Flatten the matrix and join elements with commas
    matrix_str = ','.join(map(str, matrix.flatten()))
    return matrix_str

def get_forces(matrix_str):
    """
    Convert a string representation back to a 3x3 numpy matrix.

    Parameters:
        matrix_str (str): A string representation of a 3x3 matrix.

    Returns:
        np.ndarray: A 3x3 numpy array.
    """
    # Split the string by commas and convert to float
    elements = list(map(float, matrix_str.split(',')))
    
    if len(elements) != 9:
        raise ValueError("Input string must represent a 3x3 matrix with 9 elements.")
    
    # Reshape the list back into a 3x3 matrix
    matrix = np.array(elements).reshape((3, 3))
    return matrix

def string_objects(list_of_vectors):
    """
    Convert a list of lists (each containing a 3x1 numpy vector and an integer) to a string representation.

    Parameters:
        list_of_vectors (list): A list of lists, each containing a 3x1 numpy array and an integer.

    Returns:
        str: A string representation of the list of vectors.
    """
    string_elements = []
    
    for item in list_of_vectors:
        vector, integer, isDrone = item
        
        if not isinstance(vector, np.ndarray) or vector.shape != (3, 1):
            raise ValueError("Each vector must be a 3x1 numpy array.")
        if not isinstance(integer, int):
            raise ValueError("The second element of each list must be an integer.")
        
        # Convert vector to a string of comma-separated values
        vector_str = ','.join(map(str, vector.flatten()))
        # Combine vector string with integer
        item_str = f"{vector_str}_{integer}_{isDrone}"
        string_elements.append(item_str)
    
    # Join all elements with a vertical bar to separate different list items
    result_str = '|'.join(string_elements)
    return result_str

def get_objects(string):
    """
    Convert a string representation back to a list of lists, each containing a 3x1 numpy vector and an integer.

    Parameters:
        string (str): A string representation of the list of vectors.

    Returns:
        list: A list of lists, each containing a 3x1 numpy array and an integer.
    """
    if string == '':
        return []
    
    # Split the string by vertical bar to separate different list items
    list_elements = string.split('|')
    result_list = []
    
    for element in list_elements:
        # Split the element by semicolon to separate vector and integer
        vector_str, integer_str, isDrone_str = element.split('_')
        
        # Convert vector string back to a list of floats
        vector_elements = list(map(float, vector_str.split(',')))
        
        if len(vector_elements) != 3:
            raise ValueError("Each vector string must represent a 3x1 vector with 3 elements.")
        
        # Reshape to form a 3x1 vector
        vector = np.array(vector_elements).reshape((3, 1))
        # Convert integer string back to integer
        integer = int(integer_str)
        isDrone = int(isDrone_str)
        
        # Append to result list
        result_list.append([vector, integer, isDrone])
    
    return result_list
