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
        self.rl_count = 0
        self.wheel_speeds = ros.Publisher(
            TOPIC['WHEEL_TWIST'], Float32MultiArray, queue_size=10)
        self.DL = DriveLine(r=5.0, L=19.5 / 2.0)
        self.base_sp = 8.0
        self.w1 = self.base_sp
        self.w2 = self.base_sp

    def init_node(self):
        """Perform custom Node initialization."""
        ros.Subscriber(TOPIC['LANE_CENTROID'], Float32, self.__correctPath)
        ros.Subscriber(TOPIC['POINT_OF_INTEREST'], String,
                       self.__determineState)

    def __determineState(self, msg):
        """Handle a Point of Interest notification.

        This determines the robot's current state.

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        if msg.data == POI['STOPLIGHT'] and self.state == State.ON_PATH and self.rl_count < 2:
            self.state = State.STOPPING

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
        """Handle the current state of our robot.

        Returns wheel velocities. Unfortunately this is spaghetti code. It was
        that or one long, ugly if/elif branch with sub if/elif branches.
        """
        if self.state == State.START:
            self.state = State.ON_PATH
            return self.DL.calcWheelSpeeds(self.base_sp, self.base_sp, 0.0)
        elif self.state == State.STOPPING:
            return self.__stopStateHandler(error)
        elif self.state == State.STOPPED:
            return self.__stopStateHandler(error)
        elif self.state == State.ON_PATH:
            # Adjust based on camera
            return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
        elif self.state == State.END:
            print('END')
            return self.DL.calcWheelSpeeds(0.0, 0.0, 0.0)
        else:
            print('WHY ARE WE HERE')
            print('error: ', error)
            print('state: ', self.state)
            return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
            # elif self.state == State.ORIENT
            # elif self.state == State.GRAPH

    def __stopStateHandler(self, error):
        """Handle the stopped state of our robot."""
        if self.state == State.STOPPING:
            sleep(1)
            self.state = State.STOPPED
            return self.DL.calcWheelSpeeds(0.0, 0.0, 0.0)
        elif self.state == State.STOPPED:
            sleep(2)
            if self.rl_count == 0:
                self.rl_count = 1
                self.state = State.ON_PATH
            else:
                self.rl_count = 2
                self.state = State.CANCER_SEARCH
            return self.DL.calcWheelSpeeds(self.base_sp, self.base_sp, 0.0)
        print('wtf you doing yo?')
        print('I\'m in state: ', self.state)
        return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
        # elif self.state == obstacle
        # elif self.state == graph
