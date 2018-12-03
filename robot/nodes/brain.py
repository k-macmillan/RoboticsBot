from __future__ import division, print_function

import rospy as ros
from std_msgs.msg import Float32, String, Float32MultiArray

from robot.common import LANE_CENTROID, POINT_OF_INTEREST, WHEEL_TWIST, State
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
        self.twist = ros.Publisher(WHEEL_TWIST, Float32MultiArray, queue_size=10)
        self.DL = DriveLine(r=5.0, L=19.5 / 2.0)
        self.w1 = 5.0
        self.w2 = 5.0

    def init_node(self):
        """Perform custom Node initialization."""
        ros.Subscriber(LANE_CENTROID, Float32, self.__generateTwist)
        ros.Subscriber(POINT_OF_INTEREST, String, self.__handleObject)

    def __handleObject(self, msg):
        """Handle a Point of Interest notification.

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        self.__determineState()

    def __determineState(self):
        """Determine what state the robot is in."""
        pass

    def __generateTwist(self, msg):
        """Process the lane centroid and control x and theta velocities.

        :param msg: The lane centroid message.
        :type msg: std_msgs.msg.Float32
        """
        if self.state == State.START:


            # Adjust based on camera
            wheels = Float32MultiArray()
            w1, w2 = self.DL.calcWheelSpeeds(self.w1, self.w2, msg.data)
            wheels.data = [w1, w2]
            self.twist.publish(wheels)
            return
        elif self.state == State.ON_PATH:

            twist = self.__roadTwist(msg)
        elif self.state == State.ON_PATH:
            twist = self.__startTwist(msg)
        elif self.state == State.CANCER_DESTROY or self.state == State.CANCER_SEARCH:
            twist = self.__parkinglotTwist(msg)
        elif self.state == State.GRAPH:
            twist = self.__graphTwist(msg)
        elif self.state == State.END:
            twist = self.__endTwist(msg)
        else:
            print('This should never happen...')
            return

        if self.verbose:
            print('Linear: (%f, %f, %f)' % (twist.linear.x, twist.linear.y,
                                            twist.linear.z))
            print('Angular: (%f, %f, %f)\n' %
                  (twist.angular.x, twist.angular.y, twist.angular.z))
        self.twist.publish(twist)

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
