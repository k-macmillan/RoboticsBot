#!/usr/bin/env python2
from __future__ import print_function
import rospy as ros
from std_msgs.msg import Float32, Int32
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
        ros.Subscriber(CAM_OBJECT, Int32, self.__handleObject)

    def __handleObject(self, msg):
        """Reads in object"""
        self.__determineState()

    def __determineState(self):
        """Sets state based on what objects are seen."""
        pass

    def __generateTwist(self, msg):
        """Processes the distance from center and publish."""
        if self.state == State.START:
            twist = self.__startTwist(msg)
        elif self.state == State.ROAD:
            twist = self.__roadTwist(msg)
        elif self.state == State.ROAD:
            twist = self.__startTwist(msg)
        elif self.state == State.P_LOT:
            twist = self.__parkinglotTwist(msg)
        else:
            twist = None

        
        # Set twist message here:
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)

        if twist is not None:
            if verbose:
                print('Linear: (%f, %f, %f)' %(twist.linear.x,
                                               twist.linear.y
                                               twist.linear.z))
                print('Angular: (%f, %f, %f)\n' %(twist.angular.x,
                                                  twist.angular.y
                                                  twist.angular.z))
            self.twist.publish(twist)

    def __startTwist(self, msg):
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        return twist

    def __roadTwist(self, msg):
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        return twist

    def __parkinglotTwist(self, msg):
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        return twist