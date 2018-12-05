from __future__ import division, print_function

import os
import select
import sys
import termios

import cv2
import rospy as ros
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32

from robot.common import TOPIC
from robot.nodes import Node


class Joystick(Node):
    """A ROS node to drive a *very* simple differential drive robot."""

    keymap = {
        'w': (1, 1),
        'a': (-1, 1),
        's': (-1, -1),
        'd': (1, -1),
        'W': (2, 2),
        'A': (-2, 2),
        'S': (-2, -2),
        'D': (2, -2),
        ' ': (0, 0),
    }

    def __init__(self, stdin):
        """Initialize the Joystick node."""
        super(Joystick, self).__init__(name='Joystick')
        self.stdin = stdin
        self.settings = None
        self.left_publisher = ros.Publisher(
            TOPIC['WHEEL_LEFT'], Int32, queue_size=1)
        self.right_publisher = ros.Publisher(
            TOPIC['WHEEL_RIGHT'], Int32, queue_size=1)
        self.camera_topic = TOPIC['CAMERA_FEED']
        self.bridge = CvBridge()

    def init_node(self):
        """Perform custom Node initialization."""
        ros.Subscriber(self.camera_topic, CompressedImage, self.image_handler)
        ros.Timer(ros.Duration(0.1), self.callback)
        sys.stdin = os.fdopen(self.stdin)
        self.settings = termios.tcgetattr(sys.stdin)
        new_attrs = self.settings[:]
        new_attrs[3] = new_attrs[3] & ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, new_attrs)

        self.usage()

    @staticmethod
    def usage():
        """Print usage statement."""
        print('wasd for half speed, WASD for full speed.')
        print('Space to stop.')

    @staticmethod
    def get_keypress():
        """Get the next available keypress from stdin."""
        select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1)

    def callback(self, event):
        """Run the joystick event loop body."""
        key = self.get_keypress()
        if key in self.keymap:
            left, right = self.keymap[key]

            # Make 1 modifier go half speed.
            left *= 50
            right *= 50

            # Add padding so that (100, 100) gets overwritten by (0, 0)
            sys.stdout.write('Driving (L, R): ({}, {})          \r'.format(
                left, right))
            sys.stdout.flush()

            msg = Int32()
            msg.data = left
            self.left_publisher.publish(msg)
            msg.data = right
            self.right_publisher.publish(msg)

    def stop(self):
        """Handle shutdown signals."""
        # You will regret the day that this fails.
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.settings)
        # Shut off the wheels.
        msg = Int32()
        msg.data = 0
        self.left_publisher.publish(msg)
        self.right_publisher.publish(msg)
        super(Joystick, self).stop()

    def image_handler(self, compressed):
        """Handle each compressed video frame.

        :param compressed: The compressed video frame.
        :type compressed: sensor_msgs.msg.CompressedImage
        """
        try:
            # Decompress the message into an openCV frame.
            bgr_frame = self.bridge.compressed_imgmsg_to_cv2(
                compressed, 'bgr8')
        except CvBridgeError as e:
            print(e)

        cv2.namedWindow('Joystick', cv2.WINDOW_NORMAL)
        cv2.imshow('Joystick', bgr_frame)
        cv2.waitKey(10)
