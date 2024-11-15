import numpy as np
from trajectory import Trajectory, TrajectoryState


class PolynomialTrajectory(Trajectory):  # type: ignore
    def __init__(
        self, points: list[np.ndarray], average_velocity: float = 0.05
    ) -> None:
        """
        Parameters:
        - points: (N, 3) array of N waypoint coordinates in 3D
        - average_velocity: the average speed between segments
        """

        Trajectory.__init__(self)

        self.__points = np.array(points)
        self.__average_velocity = average_velocity

        assert (
            self.__points.shape[1] == 3
        ), "Invalid number of dimensions for points. Must be 3D"
        assert (
            self.__points.shape[0] > 1
        ), "Not enough points to create a trajectory. Must have at least 2 points"
        assert average_velocity > 0, "Average velocity must be greater than 0"

        (
            self.__points,
            self.__segment_duration,
            self.__start_times,
            self.__x_polynomials,
            self.__x_dot_polynomials,
            self.__x_ddot_polynomials,
        ) = self.__preprocess_trajectory()

    def __get_polynomial(self, x_i: float, x_f: float, T: float) -> np.ndarray:
        """
        Return fully constrained polynomial coefficients from xi to xf in
        time interval [0,T]. Low derivatives are all zero at the endpoints.
        """

        A = np.array(
            [
                [0, 0, 0, 0, 0, 1],
                [T**5, T**4, T**3, T**2, T, 1],
                [0, 0, 0, 0, 1, 0],
                [5 * T**4, 4 * T**3, 3 * T**2, 2 * T, 1, 0],
                [0, 0, 0, 2, 0, 0],
                [20 * T**3, 12 * T**2, 6 * T, 2, 0, 0],
            ]
        )
        b = np.array([x_i, x_f, 0, 0, 0, 0])
        polynomial = np.linalg.solve(A, b)
        return polynomial

    def __preprocess_trajectory(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Returns:
        - points: (N, 3) array of N waypoint coordinates in 3D after
            removing sequential duplicates
        - segment_duration: (N,) array of segment durations
        - start_times: (N,) array of start times for each segment
        - x_polynomials: (N, 3, 6) array of polynomials for each segment
        - x_dot_polynomials: (N, 3, 5) array of derivative polynomials for each segment
        - x_ddot_polynomials: (N, 3, 4) array of second derivative polynomials
            for each segment
        """

        # * Remove any sequential duplicate points (or points that are too close);
        # always keep the first instance
        segment_lengths = np.linalg.norm(np.diff(self.__points, axis=0), axis=1)
        segment_mask = np.append(True, segment_lengths > 1e-3)
        points = self.__points[segment_mask]

        # * If at least two points are left, calculate segment polynomials
        if points.shape[0] >= 2:
            num_of_points = points.shape[0] - 1
            segment_duration = segment_lengths / self.__average_velocity

            x_polynomials = np.zeros((num_of_points, 3, 6))
            for i in range(num_of_points):
                for j in range(3):
                    x_polynomials[i, j, :] = self.__get_polynomial(
                        points[i, j], points[i + 1, j], segment_duration[i]
                    )

        # * Else, hard code constant polynomial at initial waypoint position
        else:
            num_of_points = 1
            segment_duration = np.zeros((num_of_points,))
            x_polynomials = np.zeros((num_of_points, 3, 6))
            x_polynomials[0, :, -1] = self.__points[0, :]

        # * Compute start times for each segment
        start_times = np.concatenate(([0], np.cumsum(segment_duration[:-1])))

        # * Compute the derivative polynomials
        x_dot_polynomials = np.zeros((num_of_points, 3, 5))
        x_ddot_polynomials = np.zeros((num_of_points, 3, 4))
        for i in range(num_of_points):
            for j in range(3):
                x_dot_polynomials[i, j, :] = np.polyder(x_polynomials[i, j, :], m=1)
                x_ddot_polynomials[i, j, :] = np.polyder(x_polynomials[i, j, :], m=2)

        return (
            points,
            segment_duration,
            start_times,
            x_polynomials,
            x_dot_polynomials,
            x_ddot_polynomials,
        )

    def update(self, t: float) -> TrajectoryState:
        """
        Given the present time, return the desired state.
        - Parameters:
            - t: time (s)
        - Returns:
            TrajectoryState object with the following fields:
            - x: desired x position (m)
            - y: desired y position (m)
            - z: desired z position (m)
            - dx: desired x velocity (m/s)
            - dy: desired y velocity (m/s)
            - dz: desired z velocity (m/s)
            - ddx: desired x acceleration (m/s^2)
            - ddy: desired y acceleration (m/s^2)
            - ddz: desired z acceleration (m/s^2)
            - yaw: desired yaw angle (rad)
            - yaw_rate: desired yaw rate (rad/s)
        """

        # TODO: handle yaw and yaw_rate

        position = np.zeros((3,))
        velocity = np.zeros((3,))
        acceleration = np.zeros((3,))
        yaw = 0
        yaw_rate = 0

        # * Find interval index i and time within interval t
        t = np.clip(
            t,
            self.__start_times[0],
            self.__start_times[-1] + self.__segment_duration[-1],
        )
        for i in range(len(self.__start_times)):
            if self.__start_times[i] + self.__segment_duration[i] > t:
                break
        t = t - self.__start_times[i]

        # * Evaluate polynomial
        for j in range(3):
            position[j] = np.polyval(self.__x_polynomials[i, j, :], t)
            velocity[j] = np.polyval(self.__x_dot_polynomials[i, j, :], t)
            acceleration[j] = np.polyval(self.__x_ddot_polynomials[i, j, :], t)

        return TrajectoryState(
            position[0],
            position[1],
            position[2],
            velocity[0],
            velocity[1],
            velocity[2],
            acceleration[0],
            acceleration[1],
            acceleration[2],
            yaw,
            yaw_rate,
        )
