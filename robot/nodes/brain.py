#!/usr/bin/env python2
from __future__ import print_function
import rospy as ros
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3

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

        ros.Subscriber(CAM_CENTER_DIST, Float32, self.__generateTwist)

    def __generateTwist(self, msg):
        """Processes the distance from center and publish."""
        twist = Twist()
        # Set twist message here:
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)

        if verbose:
            print('Linear: (%f, %f, %f)' %(twist.linear.x,
                                           twist.linear.y
                                           twist.linear.z))
            print('Angular: (%f, %f, %f)\n' %(twist.angular.x,
                                              twist.angular.y
                                              twist.angular.z))
        self.twist.publish(twist)
