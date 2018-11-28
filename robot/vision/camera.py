from __future__ import division, print_function

import cv2
import rospy as ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import UInt8

from robot.common import State
from robot.nodes import Node


class CameraController(Node):
    """ROS node to process the live webcam feed from the robot.

    This node subscribes to:

    /geekbot/webcam/image_raw/compressed - The compressed live camera feed from
    the geekbot package running on the robot.
    /robot/state (TBD) - The current robot state. E.g., is the robot currently
    line-following, navigating the cancerous parking lot, etc.

    This node publishes:

    /robot/centroid (Float32) - The centroid of the lane contour. There well be
    no messages published on this topic if the current state is not currently
    line-following.
    /robot/encounter (String) - A string serialization of any points we encounter.
    E.g., a stoplight, or the parking lot entrance.
    """

    def __init__(self, camera_topic, state_topic, verbose=False):
        """Initialize the CameraController node with the proper topics.

        :param camera_topic: The topic publishing the compressed video feed.
        :type camera_topic: str
        :param state_topic: The topic publishing the current state.
        :type state_topic: str
        :param verbose: Whether or not to console spam with useless random info.
        :type verbose: bool
        """
        super(CameraController, self).__init__(name='CameraController')
        ros.Subscriber(camera_topic, CompressedImage, self.image_handler)
        ros.Subscriber(state_topic, UInt8, self.state_handler)

        self.verbose = verbose
        self.state = State.START
        self.bridge = CvBridge()

    def image_handler(self, compressed):
        """Handle each compressed video frame.

        Based on the current state, perform different analysis on the given image.

        :param compressed: The compressed video frame.
        :type compressed: sensor_msgs.msg.CompressedImage
        """
        try:
            # Decompress the message into an openCV frame.
            bgr_frame = self.bridge.compressed_imgmsg_to_cv2(compressed, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # Convert BGR to HSV.
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        if self.state == State.ON_PATH:
            # TODO: Implement a LaneCamera and a StoplightCamera. The LaneCamera
            # will find the lane's position in the current field of view. The
            # StoplightCamera will notify is if we are directly in front of a
            # stoplight.
            pass
        elif self.state == State.CANCER_SEARCH:
            # TODO: Possibly implement a CancerSearchCamera that searches for the
            # exit in the parking lot.
            pass
        elif self.state == State.CANCER_DESTROY:
            # TODO: Implement a CancerDestroyCamera that somehow uses a potential
            # function to compute a gradient that pushes us away from the obstacles,
            # away from the edge, and towards the exit. The only thing it needs
            # to return is the gradient at the current position; there's no strict
            # need to localize.
            pass
        elif self.state == State.ORIENT:
            # TODO: Implement an OrientationCamera that helps the Brain position
            # itself at an intersection so that the robot is facing a road.
            pass
        elif self.state == State.GRAPH:
            # TODO: Implement a GraphCamera that identifies nodes in the graph.
            pass

        cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
        cv2.imshow('Original', bgr_frame)

        cv2.waitKey(10)

    def state_handler(self, state):
        """Handle each state update.

        :param state: The new state update.
        :type state: a std_msgs.msg.UInt8 msg containing a robot.common.State enum.
        """
        state = State(state.data)
        if self.verbose:
            print('Receiving state update', self.state, '->', state)
        self.state = state
