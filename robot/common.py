from enum import Enum

WHEEL_LEFT  = '/geekbot/left_wheel'
WHEEL_RIGHT = '/geekbot/right_wheel'
WHEEL_TWIST = '/geekbot/wheel_twist'

CAM_CENTER_DIST = '/geekbot/camera_center_dist'
CAM_OBJECT      = '/geekbot/camera_object'

class State(Enum):
    START = 0
    ON_PATH = 1
    STOPPED = 2
    CANCER_SEARCH = 3
    CANCER_DESTROY = 4
    ORIENT = 5
    GRAPH = 6
    END = 7
