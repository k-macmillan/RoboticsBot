from .common import TOPIC
from .nodes import Brain, NodeManager, Wheels
from .vision import CameraController


class Robot(object):
    """A class to assemble all of the ROS nodes together in one happy family."""

    def __init__(self, verbose):
        """Initialize the robot.

        :param verbose: Be very passionate about robotics.
        :type verbose: bool
        """
        self.verbose = verbose
        self.nm = NodeManager()
        self.initNodes()

    def initNodes(self):
        """Add each node to the node manager."""
        self.nm.add_node(Wheels())
        self.nm.add_node(Brain(verbose=self.verbose))
        self.nm.add_node(CameraController(TOPIC['CAMERA_FEED'],
                                          TOPIC['ROBOT_STATE'],
                                          verbose=self.verbose))

    def start(self):
        """Start the robot."""
        self.nm.spin()
