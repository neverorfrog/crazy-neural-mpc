from dataclasses import dataclass, field
from typing import List

from omegaconf import MISSING


@dataclass
class CrazyflieConfig:
    name: str = MISSING
    uri: str = MISSING
    height: float = MISSING
    multiranger: bool = False


@dataclass
class SwarmConfig:
    dt: float = field(default=0.01)
    state_publisher_rate: float = field(default=10.0)
    led_publisher_rate: float = field(default=1.0)
    velocity_publisher_rate: float = field(default=1.0)
    crazyflies: List[CrazyflieConfig] = field(
        default_factory=list[CrazyflieConfig]
    )
