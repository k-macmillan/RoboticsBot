from enum import Enum

TOPIC = {
    # Control the left wheel speed.
    'WHEEL_LEFT': '/geekbot/left_wheel',
    # Control the right wheel speed.
    'WHEEL_RIGHT': '/geekbot/right_wheel',
    # Angular and linear velocities in the local reference frame.
    'WHEEL_TWIST': '/geekbot/wheel_twist',
    # Camera feed from the robot.
    'CAMERA_FEED': '/geekbot/webcam/image_raw/compressed',
    # IR sensor feed from the robot.
    'IR_FEED': '/geekbot/ir_cm',
    # Current robot state.
    'ROBOT_STATE': '/geekbot/state',
    # Lane position in the viewing window.
    'LANE_CENTROID': '/geekbot/lane_centroid',
    # Notification that we've reached a point of interest.
    'POINT_OF_INTEREST': '/geekbot/encounter_interest',
    # Goal position in the viewing window.
    'GOAL_CENTROID': '/geekbot/goal_centroid',
    # Centroid of graph node.
    'NODE_CENTROID': '/geekbot/node_centroid',
}

# Camera POI strings
POI = {
    'STOPLIGHT': 'stoplight',
    'NO_STOPLIGHT': 'no stoplight',
    'EXIT_LOT': 'exit parking lot',
    'NO_EXIT_LOT': 'no exit parking lot',
    'GRAPH_NODE': 'graph node',
    'NO_GRAPH_NODE': 'no graph node',
    'OBSTACLE': 'obstacle',
    'NO_OBSTACLE': 'no obstacle',
}

GRAPH_PATH = {
    0: (0,),
    1: (0, 1),
    2: (0, 1, 2),
    3: (0, 1, 3),
    4: (0, 1, 4),
    5: (0, 1, 4, 5),
}

# Possible left turns on pruned graph.
LEFT_TURN = {
    (1, 4),
    (4, 5),
}

# Possible right turns on pruned graph.
RIGHT_TURN = {
    (1, 3),
}

# Possible front "turns" on pruned graph.
FORWARD = {
    (0, 1),
    (1, 2),
    (1, 3),
    (1, 4),
    (4, 5),
}


class State(Enum):
    """Possible robot states.

    The robot has the following states:

    ON_PATH - we're currently following the lane.
    STOPPING - process of stopping has begun or is in work.
    STOPPED - currently stopped at a stoplight.
    CANCER - we're searching for the parking lot exit.
    SPIN - Rotate 360 degrees every time we hit an obstacle to look for the
    lot exit.
    TURN - Rotate in place in a random direction until not obstructed.
    MTG - Motion to goal.
    GRAPH - We are at the lot exit.
    ORIENTING - Rotation until centered on lane exiting parking lot.
    G_ON_PATH - Following the lane out of the parking lot.
    NODE_STOPPING - Process of stopping at a node in in progress.
    NODE_STOPPED - Currently stopped at a node.
    ROTATE_LEFT - Rotating left until node is in view.
    ROTATE_RIGHT - Rotating right until node is in view.
    FORWARD - Drive forward until at another node.
    END - we're done. Queue the less (more?)-heavy drinking.
    """

    # Line follow
    ON_PATH = 1
    STOPPING = 2
    STOPPED = 3

    # Parking Lot
    CANCER = 4
    SPIN = 5
    TURN = 6
    MTG = 7

    # Graph
    GRAPH = 8
    ORIENTING = 9
    G_ON_PATH = 10
    NODE_STOPPING = 11
    NODE_STOPPED = 12
    ROTATE_LEFT = 13
    ROTATE_RIGHT = 14
    FORWARD = 15
    END = 16
