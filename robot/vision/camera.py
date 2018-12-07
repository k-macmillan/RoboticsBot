from __future__ import division, print_function

import cv2
import rospy as ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, String, UInt8

from robot.common import TOPIC, State
from robot.nodes import Node

from .camera_goal import GoalCamera
from .camera_lane import LaneCamera
from .camera_node import NodeCamera
from .camera_obstacle import ObstacleCamera
from .camera_stoplight import StoplightCamera


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
    /robot/encounter (String) - A string serialization of any points we
    encounter. E.g., a stoplight, or the parking lot entrance.
    """

    # How much do we blur the image
    BLUR_KERNEL = (5, 5)

    def __init__(self, camera_topic, state_topic, verbose=False):
        """Initialize the CameraController node with the proper topics.

        :param camera_topic: The topic publishing the compressed video feed.
        :type camera_topic: str
        :param state_topic: The topic publishing the current state.
        :type state_topic: str
        :param verbose: Whether or not to console spam with useless random
        info.
        :type verbose: bool
        """
        super(CameraController, self).__init__(name='CameraController')

        self.camera_topic = camera_topic
        self.state_topic = state_topic

        self.verbose = verbose
        # Force the camera's state during testing:wq
        self.state = State.ON_PATH
        self.bridge = CvBridge()

        poi_pub = ros.Publisher(
            TOPIC['POINT_OF_INTEREST'], String, queue_size=1)
        lane_pub = ros.Publisher(TOPIC['LANE_CENTROID'], Float32, queue_size=1)
        exit_pub = ros.Publisher(TOPIC['GOAL_CENTROID'], Float32, queue_size=1)
        node_pub = ros.Publisher(TOPIC['NODE_CENTROID'], Float32, queue_size=1)

        self.lane_camera = LaneCamera(lane_pub, verbose=False)
        self.stoplight_cam = StoplightCamera(poi_pub, verbose=False)
        self.obstacle_cam = ObstacleCamera(poi_pub, verbose=False)
        self.exit_cam = GoalCamera(exit_pub, poi_pub, verbose=verbose)
        self.node_cam = NodeCamera(node_pub, poi_pub, verbose=False)

    def init_node(self):
        """Perform custom Node initialization."""
        # We only want the subscribers running in the Node's process, not the
        # parent's too...
        ros.Subscriber(self.camera_topic, CompressedImage, self.image_handler)
        ros.Subscriber(self.state_topic, UInt8, self.state_handler)

    def image_handler(self, compressed):
        """Handle each compressed video frame.

        Based on the current state, perform different analysis on the given
        image.

        :param compressed: The compressed video frame.
        :type compressed: sensor_msgs.msg.CompressedImage
        """
        try:
            # Decompress the message into an openCV frame.
            bgr_frame = self.bridge.compressed_imgmsg_to_cv2(
                compressed, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # Convert BGR to HSV.
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        # Blur the image before doing anything.
        hsv_frame = cv2.GaussianBlur(hsv_frame, self.BLUR_KERNEL, 0)

        if self.state == State.ON_PATH or                                     \
           self.state == State.G_ON_PATH or                                   \
           self.state == State.ORIENTING or                                   \
           self.state == State.STOPPING:
            self.lane_camera.process_image(hsv_frame)

        if self.state == State.ON_PATH or                                     \
           self.state == State.STOPPING:
            self.stoplight_cam.process_image(hsv_frame)

        if self.state == State.CANCER or                                      \
           self.state == State.SPIN or                                        \
           self.state == State.TURN:
            self.obstacle_cam.process_image(hsv_frame)

        if self.state == State.CANCER or                                      \
           self.state == State.SPIN or                                        \
           self.state == State.MTG:
            self.exit_cam.process_image(hsv_frame)

        if self.state == State.G_ON_PATH or                                   \
           self.state == State.END or                                         \
           self.state == State.NODE_STOPPING or                               \
           self.state == State.NODE_STOPPED or                                \
           self.state == State.ROTATE_LEFT or                                 \
           self.state == State.ROTATE_RIGHT or                                \
           self.state == State.ORIENTING or                                   \
           self.state == State.FORWARD:
            self.node_cam.process_image(hsv_frame)

        if self.verbose:
            cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
            cv2.imshow('Camera', bgr_frame)
            cv2.waitKey(10)

    def state_handler(self, state):
        """Handle each state update.

        :param state: The new state update.
        :type state: a std_msgs.msg.UInt8 msg containing a robot.common.State
        enum.
        """
        state = State(state.data)
        self.state = state

    def stop(self):
        """Destroy any open OpenCV windows before terminating this node."""
        if self.verbose:
            cv2.destroyAllWindows()
        super(CameraController, self).stop()
