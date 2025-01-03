from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class TrajectoryState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    ddx: float = 0.0
    ddy: float = 0.0
    ddz: float = 0.0
    yaw: float = 0.0
    yaw_rate: float = 0.0


class Trajectory(ABC):
    def __init__(self) -> None:
        pass

    @abstractmethod
    def update(self, dt: float) -> TrajectoryState:
        pass
