from enum import Enum


class Direction(Enum):
    front = 0
    left = 1
    back = 2
    right = 3

class ObjectType(Enum):
    obstacle = 0
    drone = 1
    floor = 2
