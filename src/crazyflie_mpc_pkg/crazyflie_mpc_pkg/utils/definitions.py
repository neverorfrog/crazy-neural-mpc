from enum import Enum


class FlightMode(Enum):
    IDLE = 0
    TAKEOFF = 1
    LANDING = 2
    HOVER = 3
    TRAJECTORY = 4


class TrajectoryType(Enum):
    HORZ_CIRCLE = 0
    VERT_CIRCLE = 1
    TILTED_CIRCLE = 2
    LEMNISCATE = 3
    HELIX = 4


class MotorType(Enum):
    CLASSIC = 0  # https://store.bitcraze.io/products/4-x-7-mm-dc-motor-pack-for-crazyflie-2 w/ standard props
    UPGRADE = 1  # https://store.bitcraze.io/collections/bundles/products/thrust-upgrade-bundle-for-crazyflie-2-x
