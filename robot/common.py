from enum import Enum

WHEEL_LEFT = '/geekbot/left_wheel'
WHEEL_RIGHT = '/geekbot/right_wheel'
WHEEL_TWIST = '/geekbot/wheel_twist'


class State(Enum):
    START = 0
    ROAD = 1
    P_LOT = 2
    GRAPH = 3
    END = 4