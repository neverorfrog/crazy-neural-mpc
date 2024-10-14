from dataclasses import dataclass, field
from typing import List, Type, TypeVar

import yaml
from omegaconf import MISSING, OmegaConf


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


T = TypeVar("T")


def load_config(file_path: str, config_class: Type[T]) -> T:
    """
    Load configuration from a YAML file and merge it into a configuration object of the specified class.

    Args:
      file_path (str): The path to the YAML configuration file.
      config_class (Type[T]): The class of the configuration object.

    Returns:
      T: The merged configuration object.
    """
    with open(file_path, "r") as file:
        try:
            config: T = OmegaConf.structured(config_class)
            data = OmegaConf.create(yaml.safe_load(file))
            OmegaConf.unsafe_merge(config, data)
            return config
        except yaml.YAMLError as e:
            print(f"Error decoding YAML: {e}")
            return config_class()
