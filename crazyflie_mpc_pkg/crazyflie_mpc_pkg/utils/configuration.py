from dataclasses import dataclass, field

from omegaconf import MISSING


@dataclass
class QuadmodelConfig:
    mass: float = MISSING
    arm_length: float = MISSING
    Ixx: float = MISSING
    Iyy: float = MISSING
    Izz: float = MISSING
    cm: float = MISSING
    tau: float = MISSING


@dataclass
class CostWeights:
    x: float = MISSING
    y: float = MISSING
    z: float = MISSING
    vx: float = MISSING
    vy: float = MISSING
    vz: float = MISSING
    roll: float = MISSING
    pitch: float = MISSING
    yaw: float = MISSING
    roll_c: float = MISSING
    pitch_c: float = MISSING
    yaw_c: float = MISSING
    thrust: float = MISSING


@dataclass
class InputConstraints:
    roll_min: float = MISSING
    roll_max: float = MISSING
    pitch_min: float = MISSING
    pitch_max: float = MISSING
    yaw_min: float = MISSING
    yaw_max: float = MISSING
    thrust_min: float = MISSING
    thrust_max: float = MISSING


@dataclass
class StateConstraints:
    x_min: float = MISSING
    x_max: float = MISSING
    y_min: float = MISSING
    y_max: float = MISSING
    z_min: float = MISSING
    z_max: float = MISSING
    vx_min: float = MISSING
    vx_max: float = MISSING
    vy_min: float = MISSING
    vy_max: float = MISSING
    vz_min: float = MISSING
    vz_max: float = MISSING
    roll_min: float = MISSING
    roll_max: float = MISSING
    pitch_min: float = MISSING
    pitch_max: float = MISSING
    yaw_min: float = MISSING
    yaw_max: float = MISSING


@dataclass
class MpcConfig:
    dt: float = MISSING
    horizon: int = MISSING
    quadmodel: QuadmodelConfig = field(default_factory=QuadmodelConfig)
    cost_weights: CostWeights = field(default_factory=CostWeights)
    input_constraints: InputConstraints = field(
        default_factory=InputConstraints
    )
    state_constraints: StateConstraints = field(
        default_factory=StateConstraints
    )
