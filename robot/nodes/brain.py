#!/usr/bin/env python2
from __future__ import print_function
import rospy as ros
from geometry_msgs.msg import Twist

from robot.nodes import Node
from robot.common import WHEEL_TWIST, State


class Brain(Node):
    """A ROS Node to handle the brain of our robot."""

    def __init__(self, verbose=False):
        """Initializes publishers and subsribers"""
        super(Brain, self).__init__(name='Brain')
        self.verbose = verbose
        self.state = State.START
        self.twist = ros.Publisher(WHEEL_TWIST,
                                   Twist,
                                   queue_size=10)

        #ros.Subscriber(WHEEL_TWIST, Twist, self.__generateTwist)

    def __generateTwist(self, msg):
        """Processes the Twist message and sends that to the publish method."""
        pass
