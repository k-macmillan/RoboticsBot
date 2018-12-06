from __future__ import division, print_function

import random
from time import sleep

import rospy as ros
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8

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
        self.state = State.CANCER
        self.last_state = State.CANCER
        self.last_error = 1000.0
        self.rl_count = 0
        self.timer_counter = 0
        self.timer = None
        self.obst_rot = False
        self.spun = False

        # POI updates & state stuff
        self.obstacle = False
        self.exit = False
        self.goal_error = 0.0

        self.wheel_speeds = ros.Publisher(
            TOPIC['WHEEL_TWIST'], Float32MultiArray, queue_size=1)
        self.state_pub = ros.Publisher(
            TOPIC['ROBOT_STATE'], UInt8, queue_size=1)
        self.DL = DriveLine(r=5.0, L=19.5 / 2.0)
        self.base_sp = 8.0
        self.w1 = self.base_sp
        self.w2 = self.base_sp

    def init_node(self):
        """Perform custom Node initialization."""
        # ros.Subscriber(TOPIC['LANE_CENTROID'], Float32, self.__correctPath)
        ros.Subscriber(TOPIC['GOAL_CENTROID'], Float32, self.__topicGoal)
        ros.Subscriber(TOPIC['POINT_OF_INTEREST'],
                       String,
                       self.__topicPOI)

    def transition(self, state):
        """Transition the robot's state to that given.

        :param state: state
        :type state: robot.common.State
        """
        if self.verbose:
            print(self.state, '->', state)
        self.state = state
        msg = UInt8()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def __topicGoal(self, msg):
        self.goal_error = msg.data

    def __topicPOI(self, msg):
        """Handle a Point of Interest notification.

        This determines the robot's current state.

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        if msg.data == POI['STOPLIGHT'] and\
                self.state == State.ON_PATH and self.rl_count < 2:
            self.transition(State.STOPPING)
            if self.rl_count == 1:
                w1, w2 = self.__stateHandler(0)
                # We only want to change self.w if not in CANCER
                if self.state != State.CANCER:
                    self.w1 = w1
                    self.w2 = w2
                    wheels = Float32MultiArray()
                    wheels.data = [self.w1, self.w2]
                    self.wheel_speeds.publish(wheels)
        # Paradigm shift in state handling
        elif msg.data == POI['OBSTACLE']:
            self.obstacle = True
        elif msg.data == POI['NO_OBSTACLE']:
            self.obstacle = False
        elif msg.data == POI['EXIT_LOT']:
            self.exit = True
        elif msg.data == POI['NO_EXIT_LOT']:
            self.exit = False

    def __stateTimer(self):
        if self.state_timer is None:
            self.state_timer = ros.Timer(
                ros.Duration(secs=0.01), self.__stateHandler2)

    def __stateHandler2(self):
        if self.state == State.CANCER:
            self.__cancerState()
        elif self.state == State.SPIN:
            self.__spinState()
        elif self.state == State.TURN:
            self.__turnState()
        elif self.state == State.MTG:
            self.__mtgState()
        elif self.state == State.GRAPH:
            pass

    def __cancerState(self):
        if self.obstacle:
            self.transition(State.SPIN)
        elif self.exit:
            self.transition(State.MTG)
        else:
            self.__setWheels(self.base_sp, self.base_sp)

    def __spinState(self):
        pass

    def __turnState(self):
        pass

    def __mtgState(self):

        pass

    def __setWheels(self, w1=None, w2=None):
        if w1 is None or w2 is None:
            w1 = self.w1
            w2 = self.w2
        wheels = Float32MultiArray()
        wheels.data = [w1, w2]
        self.wheel_speeds.publish(wheels)

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
        elif self.state == State.CANCER:
            # Begin state timer that we will use from here on out
            self.__stateTimer()
            return 0.0, 0.0

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
                self.transition(State.CANCER)
            return self.DL.calcWheelSpeeds(self.base_sp, self.base_sp, 0.0)
        print('I\'m in state: ', self.state)
        return self.DL.calcWheelSpeeds(self.w1, self.w2, error)
        # elif self.state == obstacle
        # elif self.state == graph

    def __obstacleAvoidance(self):
        if not self.spun:
            self.spun = True
            self.last_state = self.state
            self.transition(State.SPIN)
            self.__forceCorrectPath(0)
            return 0.0, 0.0

        if not self.obst_rot:
            self.obst_rot = True
            if bool(random.getrandbits(1)):
                self.multiplier = -1
            else:
                self.multiplier = 1
        # print('Multiplier: ', self.multiplier)
        return self.DL.calcWheelSpeeds(self.w1 * self.multiplier,
                                       self.w2 * -self.multiplier, 0.0)

    def __start_timer(self):
        if self.timer is None:
            self.timer = ros.Timer(
                ros.Duration(secs=0.01), self.__timerCallback)

    def __timerCallback(self, event):
        if self.timer_counter > 365:
            self.transition(State.OBSTACLE)
            self.__timerShutdown()
        elif self.last_error < 1000.0:
            self.transition(State.CANCER)
            self.__timerShutdown()
        else:
            self.timer_counter += 1

    def __timerShutdown(self):
        self.timer.shutdown()
        self.timer_counter = 0
        self.timer = None
        self.w1 = self.base_sp
        self.w2 = self.base_sp
        wheels = Float32MultiArray()
        wheels.data = [0.0, 0.0]
        self.wheel_speeds.publish(wheels)
