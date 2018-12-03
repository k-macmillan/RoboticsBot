from __future__ import division, print_function

import rospy as ros
from std_msgs.msg import Int32, Float32MultiArray

from robot.common import WHEEL_LEFT, WHEEL_RIGHT, WHEEL_TWIST
from robot.nodes import Node


class Wheels(Node):
    """A ROS Node to handle the wheels of our robot."""

    def __init__(self, verbose=False):
        """Initialize the ROS Node."""
        super(Wheels, self).__init__(name='Wheels')
        self.verbose = verbose
        self.left_pub = ros.Publisher(WHEEL_LEFT, Int32, queue_size=10)
        self.right_pub = ros.Publisher(WHEEL_RIGHT, Int32, queue_size=10)

    def init_node(self):
        """Perform custom Node initialization."""
        ros.Subscriber(WHEEL_TWIST, Float32MultiArray, self.__processTwist)

    def __processTwist(self, msg):
        """Process the Twist message and sends that to the publish method."""
        self.__publishWheels(msg.data[0], msg.data[1])

    def __publishWheels(self, left, right):
        """Publish the left hand right wheel speeds."""
        msg = Int32()
        upper = 10.0  # Maximum wheel speed coming from left/right
        msg.data = int((left / upper) * 100)
        self.left_pub.publish(msg)
        msg.data = int((right / upper) * 100)
        self.right_pub.publish(msg)

    def stop(self):
        """Handle the shutdown signal from the NodeManager."""
        super(Wheels, self).stop()
        msg = Int32()
        msg.data = 0
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)
