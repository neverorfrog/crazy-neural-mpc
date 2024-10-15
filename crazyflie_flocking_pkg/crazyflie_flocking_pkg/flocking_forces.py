import numpy as np

from crazyflie_flocking_pkg.crazyflie_flocking_pkg.utils.configuration import (
    FlockingConfig,
)
from crazyflie_flocking_pkg.utils.definitions import Direction


class ForcesGenerator:
    def __init__(self, config: FlockingConfig):
        self.config = config

    def get_forces(
        self, self_pos, self_yaw, robot_positions, obs_dist_dir, v_mig
    ):
        # Initialization
        f_inter_robot = np.zeros((3, 1))
        f_obstacle = np.zeros((3, 1))
        f_migration = np.zeros((3, 1))

        # Inter-robot forces, formula (2)
        for p in robot_positions:
            p = np.reshape(p, (3, 1))
            distance = np.linalg.norm(p - self_pos) - 2 * self.radius
            if distance > 2 * self.d_eq:
                continue
            u_ij = normalize(p - self_pos)
            f_inter_robot += self.k_r * (distance - self.d_eq) * u_ij
        f_inter_robot[2] = 0

        # Obstacle avoidance forces, formula (3)
        for o in obs_dist_dir:
            dist = o[0]
            dir = o[1]

            distance_ik = dist - self.radius
            d_0 = self.max_vis_objs - self.radius

            R = np.array(
                [
                    [np.cos(self_yaw), -np.sin(self_yaw), 0],
                    [np.sin(self_yaw), np.cos(self_yaw), 0],
                    [0, 0, 1],
                ]
            )

            if dir == Direction.front:
                u_ik = R @ np.array([1, 0, 0])
            elif dir == Direction.back:
                u_ik = R @ np.array([-1, 0, 0])
            elif dir == Direction.left:
                u_ik = R @ np.array([0, 1, 0])
            elif dir == Direction.right:
                u_ik = R @ np.array([0, -1, 0])

            u_ik = np.reshape(u_ik, (3, 1))
            # f_obstacle += -self.k_o * (1/(distance_ik)**2)* u_ik               # f_obs originale
            # f_obstacle += -self.k_o * (1/(distance_ik)**2 - 1/(d_0)**2)* u_ik  # f_obs continua
            f_obstacle += (
                -self.k_o * (1 / (distance_ik) - 1 / (d_0)) ** 2 * u_ik
            )  # f_obs APF
        f_obstacle[2] = 0

        # Migration force, formula (4)
        v_mig = np.reshape(v_mig, (3, 1))
        f_migration = self.k_m * v_mig

        # Alignment control (Self-organized flocking with a mobile robot swarm: a novel motion control method)
        """f_alignment = np.array([cos(self_yaw), sin(self_yaw), 0])
        for theta_i in robot_orientations:
            e_i_theta = np.array([cos(theta_i), sin(theta_i), 0])
            f_alignment += e_i_theta
        f_alignment = normalize(f_alignment)
        f_alignment = self.k_al * np.reshape(np.array(f_alignment), (3,1))"""

        return np.hstack(
            (f_inter_robot, f_obstacle, f_migration)
        )  # , f_alignment))

    def force_limit(self, force):
        force_norm = np.linalg.norm(force)
        force_norm_constrained = constrained_value(
            force_norm, 0.0, self.max_force
        )
        return force * (force_norm_constrained / force_norm)

    def compute_velocities(self, force, u_i, gamma=0):
        vector_orthogonal_to_plane = np.array([[0], [0], [1]])

        # Linear speed, formulas (5), (7)
        v_scalar = self.k_l * (np.dot(force, u_i))
        v_scalar = constrained_value(
            v_scalar, self.v_min, self.v_max
        )  # v_scalar constrained in v-min and v_max
        v = v_scalar  # * u_i
        # v = self.k_l * force
        # v = self.constraint_vel(v)

        # Angular speed, formulas (6), (8)
        # u_i_orthogonal computed as vector orthogonal to u_i and vector_orthogonal_to_plane (z axis)
        u_i_orthogonal = np.cross(vector_orthogonal_to_plane[:, 0], u_i[:, 0])
        omega_scalar = self.k_a * (np.dot(force, u_i_orthogonal))
        omega_scalar = constrained_value(
            omega_scalar, self.omega_min, self.omega_max
        )  # omega_scalar constrained in omega_min and omega_max
        omega = omega_scalar * vector_orthogonal_to_plane  # omega as 3D vector

        # Omega APF di AMR
        # omega_scalar = self.k_a * (atan2(force[1], force[0])- gamma)
        return [v, omega_scalar]

    def constraint_vel(self, v):
        normalizer = np.linalg.norm(v) / self.v_max

        if normalizer > 1:
            return v / normalizer
        else:
            return v


def normalize(vector):
    # Given a vector, return the versor
    norm_vector = np.linalg.norm(vector)
    if norm_vector < 1e-14:
        print("Error: vector 0, impossible to normalize")
        return vector
    return vector / norm_vector


def constrained_value(val, val_min, val_max):
    # If val is out of range [v_min; v_max], return one of the 2 extremes
    return float(min(max(val_min, val), val_max))
