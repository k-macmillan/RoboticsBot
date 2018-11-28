from __future__ import division, print_function

import rospy as ros
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, String

from robot.common import LANE_CENTROID, POINT_OF_INTEREST, WHEEL_TWIST, State
from robot.nodes import Node


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

        ros.Subscriber(LANE_CENTROID, Float32, self.__generateTwist)
        ros.Subscriber(POINT_OF_INTEREST, String, self.__handleObject)

    def __handleObject(self, msg):
        """Reads in object"""
        self.__determineState()

    def __determineState(self):
        """Sets state based on what objects are seen."""
        pass

    def __generateTwist(self, msg):
        """Processes the distance from center and publish Twist based on state."""
        if self.state == State.START:
            twist = self.__startTwist(msg)
        elif self.state == State.ROAD:
            twist = self.__roadTwist(msg)
        elif self.state == State.ROAD:
            twist = self.__startTwist(msg)
        elif self.state == State.P_LOT:
            twist = self.__parkinglotTwist(msg)
        elif self.state == State.GRAPH:
            twist = self.__graphTwist(msg)
        elif self.state == State.END:
            twist = self.__endTwist(msg)
        else:
            print('This should never happen...')
            return

        if self.verbose:
            print('Linear: (%f, %f, %f)' %(twist.linear.x,
                                           twist.linear.y,
                                           twist.linear.z))
            print('Angular: (%f, %f, %f)\n' %(twist.angular.x,
                                              twist.angular.y,
                                              twist.angular.z))
        self.twist.publish(twist)

    def __startTwist(self, msg):
        twist = Twist()
        twist.linear = Vector3(255, 0, 0)
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

    def __graphTwist(self, msg):
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        return twist

    def __endTwist(self, msg):
        """In our end-goal state, stop the robot."""
        twist = Twist()
        twist.linear = Vector3(0, 0, 0)
        twist.angular = Vector3(0, 0, 0)
        return twist
