import numpy as np
from trajectory import Trajectory, TrajectoryState


class CircularTrajectory(Trajectory):  # type: ignore
    def __init__(
        self,
        center: list[float] = [0, 0, 0],
        radius: float = 1,
        freq: float = 0.01,
        yaw_bool: bool = False,
        plane: str = "XY",
        direction: str = "CCW",
    ) -> None:
        """
        Parameters:
        - center: the center of the circle (m)
        - radius: the radius of the circle (m)
        - freq: the frequency with which a circle is completed (Hz)
        - yaw_bool: determines if yaw motion is desired
        - plane: the plane with which the circle lies on, 'XY', 'YZ', or 'XZ'
        - direcition: the direction of the circle, 'CCW' or 'CW'
        """

        Trajectory.__init__(self)
        assert plane in [
            "XY",
            "XZ",
            "YZ",
        ], "Invalid Plane. Choose from XY, XZ, YZ"
        assert direction in [
            "CW",
            "CCW",
        ], "Invalid Direction. Choose from CW, CCW"

        self.__center = np.array(center)
        self.__radius = radius
        self.__freq = freq
        self.__yaw_bool = yaw_bool
        self.__plane = plane
        self.__direction = direction

        self.__omega = 2 * np.pi * self.__freq
        self.__sign = 1 if self.__direction == "CCW" else -1

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

        # * Compute position, velocity, and acceleration
        position = self.__center + np.array(
            [
                (
                    self.__radius * np.cos(self.__sign * self.__omega * t)
                    if self.__plane == "XY" or self.__plane == "XZ"
                    else 0
                ),
                (
                    self.__radius * np.sin(self.__sign * self.__omega * t)
                    if self.__plane == "XY"
                    else (
                        self.__radius * np.cos(self.__sign * self.__omega * t)
                        if self.__plane == "YZ"
                        else 0
                    )
                ),
                (
                    self.__radius * np.sin(self.__sign * self.__omega * t)
                    if self.__plane == "XZ" or self.__plane == "YZ"
                    else 0
                ),
            ]
        )

        velocity = np.array(
            [
                (
                    -self.__radius
                    * self.__sign
                    * self.__omega
                    * np.sin(self.__sign * self.__omega * t)
                    if self.__plane == "XY" or self.__plane == "XZ"
                    else 0
                ),
                (
                    self.__radius
                    * self.__sign
                    * self.__omega
                    * np.cos(self.__sign * self.__omega * t)
                    if self.__plane == "XY"
                    else (
                        -self.__radius
                        * self.__sign
                        * self.__omega
                        * np.sin(self.__sign * self.__omega * t)
                        if self.__plane == "YZ"
                        else 0
                    )
                ),
                (
                    self.__radius
                    * self.__sign
                    * self.__omega
                    * np.cos(self.__sign * self.__omega * t)
                    if self.__plane == "XZ" or self.__plane == "YZ"
                    else 0
                ),
            ]
        )

        acceleration = np.array(
            [
                (
                    -self.__radius
                    * ((self.__sign * self.__omega) ** 2)
                    * np.cos(self.__sign * self.__omega * t)
                    if self.__plane == "XY" or self.__plane == "XZ"
                    else 0
                ),
                (
                    -self.__radius
                    * ((self.__sign * self.__omega) ** 2)
                    * np.sin(self.__sign * self.__omega * t)
                    if self.__plane == "XY"
                    else (
                        -self.__radius
                        * ((self.__sign * self.__omega) ** 2)
                        * np.cos(self.__sign * self.__omega * t)
                        if self.__plane == "YZ"
                        else 0
                    )
                ),
                (
                    -self.__radius
                    * ((self.__sign * self.__omega) ** 2)
                    * np.sin(self.__sign * self.__omega * t)
                    if self.__plane == "XZ" or self.__plane == "YZ"
                    else 0
                ),
            ]
        )

        if self.__yaw_bool:
            yaw = np.pi / 4 * np.sin(np.pi * t)
            yaw_rate = np.pi * np.pi / 4 * np.cos(np.pi * t)
        else:
            yaw = 0
            yaw_rate = 0

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
