from __future__ import division, print_function

import rospy as ros
from time import sleep
from std_msgs.msg import Float32, String, Float32MultiArray

from robot.common import TOPIC, POI, State
from robot.nodes import Node, DriveLine


class Brain(Node):
    """A ROS Node to handle the brain of our robot."""

    def __init__(self, verbose=False):
        """Initialize the Brain node.

        :param verbose: How passionate should the Brain be?, defaults to False
        :param verbose: bool, optional
        """
        super(Brain, self).__init__(name='Brain')
        self.verbose = verbose
        self.state = State.START
        self.last_state = State.START
        self.rl_count = 0
        self.wheel_speeds = ros.Publisher(TOPIC['WHEEL_TWIST'],
                                          Float32MultiArray,
                                          queue_size=10)
        self.DL = DriveLine(r=5.0, L=19.5 / 2.0)
        self.w1 = 10.0
        self.w2 = 10.0

    def init_node(self):
        """Perform custom Node initialization."""
        ros.Subscriber(TOPIC['LANE_CENTROID'], Float32, self.__correctPath)
        ros.Subscriber(TOPIC['POINT_OF_INTEREST'], String, self.__determineState)

    def __determineState(self, msg):
        """Handle a Point of Interest notification. Determines state

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        if poi.data == POI['STOPLIGHT']:
            self.last_state = self.state
            self.state = State.STOPPED

    def __correctPath(self, msg):
        """Process the lane centroid and control x and theta velocities.

        :param msg: The lane centroid message.
        :type msg: std_msgs.msg.Float32
        """
        self.w1, self.w2 = self.__stateHandler(msg.data)

        wheels = Float32MultiArray()
        wheels.data = [self.w1, self.w2]
        self.wheel_speeds.publish(wheels)

    def __stateHandler(self, error):
        """ Handles the current state of our robot. Returns wheel velocities. 
            Unfortunately this is spaghetti code. It was that or one long, ugly 
            if/elif branch with sub if/elif branches.
        """
        if self.state == State.START:
            self.state = State.ON_PATH
            return 10.0, 10.0
        elif self.state == State.STOPPED:
            return self.__stopStateHandler(error)
        elif self.state == State.ON_PATH:
            # Adjust based on camera
            return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
        # elif self.state == State.ORIENT
        # elif self.state == State.GRAPH

    def __stopStateHandler(self, error):
        """Handles the stopped state of our robot"""
        if self.last_state == State.ON_PATH:
            if self.w1 > 2.0 or self.w2 > 2.0:
                # Slow down until we pass the stop instead of adding another state
                return self.DL.calcWheelSpeeds(self.w1 * 0.9, self.w2 * 0.9, error)
            self.last_state = State.STOPPED
            return 0.0, 0.0
        elif self.last_state == State.STOPPED:
            # Could be red light (rl), obstacle, graph
            if self.rl_count == 0:
                sleep(1.5)
                self.rl_count += 1
                self.state = State.ON_PATH
                self.last_state = State.STOPPED
                return 10.0, 10.0
            elif self.rl_count == 1:
                sleep(1.5)
                self.rl_count += 1
                self.state = State.CANCER_SEARCH
                self.last_state = State.STOPPED
                return 10.0, 10.0
        # elif self.last_state == obstacle
        # elif self.last_state == graph


# OBSOLETE SECTION, WAITING ON PARTNER APPROVAL FOR DELETION

    def __startTwist(self, msg):
        """Set the local velocities in the START state.

        :param msg: The lane centroid.
        :type msg: std_msgs.msg.Float32
        :return: The desired x and theta velocities in the local reference frame
        :rtype: geometry_msgs.msg.Twist
        """
        twist = Twist()
        twist.linear.x = 255
        twist.angular.z = 0
        return twist

    def __roadTwist(self, msg):
        """Set the local velocities in the ON_PATH state.

        :param msg: The lane centroid.
        :type msg: std_msgs.msg.Float32
        :return: The desired x and theta velocities in the local reference frame
        :rtype: geometry_msgs.msg.Twist
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        return twist

    def __parkinglotTwist(self, msg):
        """Set the local velocities in the CANCER_* states.

        :param msg: The lane centroid.
        :type msg: std_msgs.msg.Float32
        :return: The desired x and theta velocities in the local reference frame
        :rtype: geometry_msgs.msg.Twist
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        return twist

    def __graphTwist(self, msg):
        """Set the local velocities in the GRAPH state.

        :param msg: The lane centroid.
        :type msg: std_msgs.msg.Float32
        :return: The desired x and theta velocities in the local reference frame
        :rtype: geometry_msgs.msg.Twist
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        return twist

    def __endTwist(self, msg):
        """Set the local velocities in the END state.

        :param msg: The lane centroid.
        :type msg: std_msgs.msg.Float32
        :return: The desired x and theta velocities in the local reference frame
        :rtype: geometry_msgs.msg.Twist
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        return twist
