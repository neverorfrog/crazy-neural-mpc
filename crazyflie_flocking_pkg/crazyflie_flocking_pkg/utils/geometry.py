import numpy as np


def point_line_distance(point, line_point, direction_vector):
    """
    Calculates the minimum distance between a point and a line in 3D space.

    :param point: Coordinates of the point (x, y, z)
    :param line_point: A point on the line (x0, y0, z0)
    :param direction_vector: Direction vector of the line (a, b, c) with magnitude 1
    :return: Minimum distance between the point and the line
    """
    # Vectors
    P = np.array(point)
    P0 = np.array(line_point)
    v = np.array(direction_vector)

    # Vector P0P
    P0P = P - P0

    # Cross product v x P0P
    cross_product = np.cross(v, P0P)

    # Magnitude of the cross product
    cross_product_magnitude = np.linalg.norm(cross_product)

    # Minimum distance
    distance = cross_product_magnitude
    return distance


def compute_absolute_position(robot_position, drone_obj, roll, pitch, yaw):
    """
    Calcola la posizione assoluta di un ostacolo dato l'orientamento e la posizione del robot.

    Parameters:
        robot_position (np.ndarray): Un vettore 3x1 che rappresenta la posizione del robot [x, y, z].
        obstacle_distance (np.ndarray): Un vettore 3x1 che rappresenta la distanza dell'ostacolo rispetto al robot [dx, dy, dz].
        roll (float): Angolo di roll (rotazione intorno all'asse X) in radianti.
        pitch (float): Angolo di pitch (rotazione intorno all'asse Y) in radianti.
        yaw (float): Angolo di yaw (rotazione intorno all'asse Z) in radianti.
        direction (int): Direction of the obstacle (FRONT, LEFT, BACK, RIGHT)

    Returns:
        np.ndarray: Un vettore 3x1 che rappresenta la posizione assoluta dell'ostacolo.
    """

    R = get_rotation_matrix(roll, pitch, yaw)

    # Calcolo della posizione assoluta dell'ostacolo
    absolute_obstacle_position = robot_position + R @ drone_obj

    return absolute_obstacle_position


def get_rotation_matrix(roll, pitch, yaw):
    # Matrice di rotazione per roll
    R_roll = np.array(
        [
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ]
    )

    # Matrice di rotazione per pitch
    R_pitch = np.array(
        [
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ]
    )

    # Matrice di rotazione per yaw
    R_yaw = np.array(
        [
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ]
    )

    # Matrice di rotazione composta
    R = R_yaw @ R_pitch @ R_roll

    return R
