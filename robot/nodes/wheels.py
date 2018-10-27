#!/usr/bin/env python2
from __future__ import print_function
import rospy as ros
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

from robot.nodes import Node
from robot.common import *


class Wheels(Node):
    """A ROS Node to handle the wheels of our robot."""

    def __init__(self, verbose=False):
        """Initializes publishers and subsribers"""
        super(Wheels, self).__init__(name='Wheels')
        self.verbose = verbose
        self.left_pub = ros.Publisher(WHEEL_LEFT,
                                      Int32,
                                      queue_size=10)
        self.right_pub = ros.Publisher(WHEEL_RIGHT,
                                       Int32,
                                       queue_size=10)

        ros.Subscriber(WHEEL_TWIST,
                       Twist,
                       self.__processTwist)

    def __processTwist(self, msg):
        """Processes the Twist message and sends that to the publish method."""
        self.__publishWheels(0, 0)

    def __publishWheels(self, left, right):
        if self.verbose:
            print('Left: %d \tRight: %d', %(left, right))
        msg = Int32()
        msg.data = left
        self.left_pub.publish(msg)
        msg.data = right
        self.right_pub.publish(msg)

    def stop(self):
        print('Terminating Node: ', self.__name)
        msg = Int32()
        msg.data = 0
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)
