import numpy as np


def center_of_flock(robot_pos):
    center = np.zeros((3, 1))
    for pos_i in robot_pos:
        center += pos_i
    center /= len(robot_pos)
    return center


def speed_error(velocity_robots, des_speed):
    E_spd = 0
    for v_i in velocity_robots:
        E_spd += abs(np.linalg.norm(v_i) - des_speed)
    E_spd /= len(velocity_robots) * des_speed
    return E_spd


def proximity(distance_matrix, d_eq):
    proximity = 0
    for dist_i in distance_matrix:
        proximity += np.linalg.norm(dist_i)
    proximity /= len(distance_matrix)
    proximity /= d_eq
    return proximity


def min_interdistance(distance_matrix):
    min_interdist = 1000
    for line in distance_matrix:
        for dist_i in line:
            if dist_i < min_interdist:
                min_interdist = dist_i
    return min_interdist


def max_interdistance(distance_matrix):
    max_interdist = 0
    for line in distance_matrix:
        for dist_i in line:
            if dist_i > max_interdist:
                max_interdist = dist_i
    return max_interdist


def polarization(velocity_robots):
    polarization = np.zeros((3, 1))
    for v_i in velocity_robots:
        normalized_v = (
            0 if np.linalg.norm(v_i) == 0 else v_i / np.linalg.norm(v_i)
        )
        polarization += normalized_v
    polarization /= len(velocity_robots)
    return polarization


def average_velocity(velocity_robots):
    average_velocity = np.zeros((3, 1))
    for v_i in velocity_robots:
        average_velocity += v_i
    average_velocity /= len(velocity_robots)
    return average_velocity


def average_speed(velocity_robots):
    average_speed = 0.0
    for v_i in velocity_robots:
        average_speed += np.linalg.norm(v_i)
    average_speed /= len(velocity_robots)
    return average_speed
