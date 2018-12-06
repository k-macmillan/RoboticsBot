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
    # The gradient at the current location in the parking lot.
    'LOT_GRADIENT': '/geekbot/cancer/gradient',
    # Goal position in the viewing window.
    'GOAL_CENTROID': '/geekbot/goal_centroid',
}

# Camera POI strings
POI = {
    'STOPLIGHT': 'stoplight',
    'EXIT_LOT': 'exit parking lot',
    'GRAPH_NODE': 'graph node',
    'OBSTACLE': 'obstacle',
    'NO_OBSTACLE': 'no obstacle',
}


class State(Enum):
    """Possible robot states.

    The robot has states:

    START - we've just started up and are waiting to begin lane-following.
    ON_PATH - we're currently following the lane.
    STOPPING - process of stopping has begun or is in work.
    STOPPED - currently stopped. Either at a stoplight, lot exit, or node.
    TESTICULAR_CANCER - we're searching for the parking lot exit.
    OBSTACLE - found an obstacle (wall or car).
    ORIENT - we're at an intersection and are turning in place so that the road
    is directly in front of us.
    GRAPH - we're traversing the graph.
    END - we're done. Queue the less-heavy drinking.
    """

    START = 0
    ON_PATH = 1
    STOPPING = 2
    STOPPED = 3
    TESTICULAR_CANCER = 4
    OBSTACLE = 5
    ORIENT = 6
    GRAPH = 7
    END = 8
    SPIN = 9
