from __future__ import division, print_function

from time import sleep

import rospy as ros
from std_msgs.msg import Float32, Float32MultiArray, String

from robot.common import POI, TOPIC, State
from robot.nodes import DriveLine, Node


class Brain(Node):
    """A ROS Node to handle the brain of our robot."""

    def __init__(self, verbose=False):
        """Initialize the Brain node.

        :param verbose: How passionate should the Brain be?, defaults to False
        :param verbose: bool, optional
        """
        super(Brain, self).__init__(name='Brain')
        self.verbose = verbose
        self.state = State.TESTICULAR_CANCER
        self.rl_count = 0
        self.wheel_speeds = ros.Publisher(
            TOPIC['WHEEL_TWIST'], Float32MultiArray, queue_size=1)
        self.DL = DriveLine(r=5.0, L=19.5 / 2.0)
        self.base_sp = 8.0
        self.w1 = self.base_sp
        self.w2 = self.base_sp

    def init_node(self):
        """Perform custom Node initialization."""
        # ros.Subscriber(TOPIC['LANE_CENTROID'], Float32, self.__correctPath)
        ros.Subscriber(TOPIC['GOAL_CENTROID'], Float32, self.__correctPath)
        # ros.Subscriber(TOPIC['POINT_OF_INTEREST'], String,
                       # self.__determineState)


    def transition(self, state):
        """Transition the robot's state to that given.

        :param state: state
        :type state: robot.common.State
        """
        if self.verbose:
            print(self.state, '->', state)
        self.state = state

    def __determineState(self, msg):
        """Handle a Point of Interest notification.

        This determines the robot's current state.

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        if msg.data == POI['STOPLIGHT'] and self.state == State.ON_PATH and self.rl_count < 2:
            self.transition(State.STOPPING)
            if self.rl_count == 1:
                self.w1, self.w2 = self.__stateHandler(0)

                wheels = Float32MultiArray()
                wheels.data = [self.w1, self.w2]
                self.wheel_speeds.publish(wheels)
        elif msg.data == POI['OBSTACLE']:
            self.transition(State.OBSTACLE)

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
            self.transition(State.ON_PATH)
            return self.DL.calcWheelSpeeds(self.base_sp, self.base_sp, 0.0)
        elif self.state == State.STOPPING or self.state == State.STOPPED:
            return self.__stopStateHandler(error)
        elif self.state == State.ON_PATH:
            # Adjust based on camera (error)
            return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
        elif self.state == State.OBSTACLE:
            return self.__obstacleAvoidance()
        elif self.state == State.TESTICULAR_CANCER:
            # We are assuming it is impossible to reach this state if there is
            # an obstacle in front of us
            self.__spin()
            if error < 1000.0:
                # Centroid found
                return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
            else:
                # Still searching, no goal found
                return self.DL.calcWheelSpeeds(self.w1, self.w2, 0.0)
        elif self.state == State.END:
            return self.DL.calcWheelSpeeds(0.0, 0.0, 0.0)
        else:
            print('error: ', error)
            print('state: ', self.state)
            return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
            # elif self.state == State.ORIENT
            # elif self.state == State.GRAPH

    def __stopStateHandler(self, error):
        """Handle the stopped state of our robot."""
        if self.state == State.STOPPING:
            sleep(1.0)
            self.transition(State.STOPPED)
            return self.DL.calcWheelSpeeds(0.0, 0.0, 0.0)
        elif self.state == State.STOPPED:
            sleep(2.0)
            if self.rl_count == 0:
                self.rl_count = 1
                self.transition(State.ON_PATH)
            else:
                self.rl_count = 2
                self.transition(State.TESTICULAR_CANCER)
            return self.DL.calcWheelSpeeds(self.base_sp, self.base_sp, 0.0)
        print('I\'m in state: ', self.state)
        return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
        # elif self.state == obstacle
        # elif self.state == graph

    def __obstacleAvoidance(self):
        # stub return:
        return self.DL.calcWheelSpeeds(self.w1, self.w2, 0.0)

    def __spin(self):
        wheels = Float32MultiArray()
        wheels.data = [self.base_sp * 100, -self.base_sp * 100]
        self.wheel_speeds.publish(wheels)
        # How long it takes for a full 360 rotation
        sleep(3.65)
        wheels.data = [0.0, 0.0]
        self.wheel_speeds.publish(wheels)
        sleep(2)
