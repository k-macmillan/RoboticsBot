from __future__ import division, print_function

import random

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
        self.state = State.ON_PATH
        self.rl_count = 0
        self.spin_timer_counter = 0

        # Timer vars
        self.state_timer = None
        self.spin_timer = None
        self.rl_timer = None

        # POI updates & state stuff
        self.stoplight_POI = False
        self.obstacle_POI = False
        self.goal_POI = False
        self.goal_error = 0.0
        self.path_error = 0.0
        self.turn_dir = 1

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
        ros.Subscriber(TOPIC['LANE_CENTROID'], Float32, self.topicPath)
        ros.Subscriber(TOPIC['GOAL_CENTROID'], Float32, self.topicGoal)
        ros.Subscriber(TOPIC['POINT_OF_INTEREST'],
                       String,
                       self.topicPOI)

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

    def topicPath(self, msg):
        # We require a bootstrap.
        if self.state_timer is None:
            self.stateTimer()
        self.path_error = msg.data

    def topicGoal(self, msg):
        self.goal_error = msg.data

    def topicPOI(self, msg):
        """Handle a Point of Interest notification.

        This determines the robot's current state.

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        if msg.data == POI['STOPLIGHT'] and self.state == State.ON_PATH:
            self.stoplight_POI = True
        elif msg.data == POI['OBSTACLE']:
            self.obstacle_POI = True
        elif msg.data == POI['NO_OBSTACLE']:
            self.obstacle_POI = False
        elif msg.data == POI['EXIT_LOT']:
            self.goal_POI = True
        elif msg.data == POI['NO_EXIT_LOT']:
            self.goal_POI = False

    def stateHandler(self, event):
        if self.state == State.ON_PATH:
            self.pathState()
        elif self.state == State.STOPPING:
            self.stoppingState()
        elif self.state == State.STOPPED:
            self.stoppedState()
        elif self.state == State.CANCER:
            self.cancerState()
        elif self.state == State.SPIN:
            self.spinState()
        elif self.state == State.TURN:
            self.turnState()
        elif self.state == State.MTG:
            self.mtgState()
        elif self.state == State.GRAPH:
            pass

    def pathState(self):
        # Tick vs Tock
        if self.stoplight_POI and (self.rl_count == 0 or self.rl_count == 2):
            self.rlTimer()
            self.transition(State.STOPPING)
        else:
            self.w1, self.w2 = self.DL.calcWheelSpeeds(self.w1,
                                                       self.w2,
                                                       self.path_error)
            self.setWheels(self.w1, self.w2)

    def stoppingState(self):
        if self.rl_timer is None:
            self.rl_count += 1
            self.setWheels(0.0, 0.0)
            self.transition(State.STOPPED)
            self.rlTimer()

    def stoppedState(self):
        if self.rl_timer is None:
            self.rl_count += 1
            if self.rl_count == 2:
                self.transition(State.ON_PATH)
            elif self.rl_count == 4:
                self.transition(State.CANCER)

    def cancerState(self):
        if self.obstacle_POI:
            self.transition(State.SPIN)
        elif self.goal_POI:
            self.transition(State.MTG)
        else:
            self.setWheels(self.base_sp, self.base_sp)

    def spinState(self):
        self.setWheels(self.base_sp, -self.base_sp)
        self.startSpinTimer()

    def turnState(self):
        if self.obstacle_POI:
            self.setWheels(self.base_sp * self.turn_dir,
                           -self.base_sp * self.turn_dir)

    def mtgState(self):
        if self.obstacle_POI:
            self.transition(State.SPIN)
        else:
            self.w1, self.w2 = self.DL.calcWheelSpeeds(self.w1,
                                                       self.w2,
                                                       self.goal_error)
            self.setWheels(self.w1, self.w2)

    # Helper functions

    def setWheels(self, w1=None, w2=None):
        if w1 is None or w2 is None:
            w1 = self.w1
            w2 = self.w2
        wheels = Float32MultiArray()
        wheels.data = [w1, w2]
        self.wheel_speeds.publish(wheels)

    def printError(self, msg):
        for i in range(20):
            print(msg)

    # Timer section

    def stateTimer(self):
        if self.state_timer is None:
            print('Creating state timer')
            self.state_timer = ros.Timer(
                ros.Duration(secs=0.01), self.stateHandler)

    def rlTimer(self):
        if self.rl_timer is None:
            print('Creating RL timer')
            self.rl_timer = ros.Timer(
                ros.Duration(secs=1.0), self.timerRLShutdown)

    def timerRLShutdown(self, event):
        self.rl_timer.shutdown()
        self.rl_timer = None
        self.stoplight_POI = False

    def startSpinTimer(self):
        if self.spin_timer is None:
            self.spin_timer = ros.Timer(
                ros.Duration(secs=0.01), self.timerSpinCallback)

    def timerSpinCallback(self, event):
        if self.spin_timer_counter > 500:
            self.timerSpinShutdown()
            # Set turn direction and set state to TURN
            if bool(random.getrandbits(1)):
                self.turn_dir = 1
            else:
                self.turn_dir = -1
            self.transition(State.TURN)
        elif not self.obstacle_POI and self.goal_POI:
            self.timerSpinShutdown()
            self.transition(State.MTG)
        else:
            self.spin_timer_counter += 1

    def timerSpinShutdown(self):
        self.spin_timer.shutdown()
        self.spin_timer = None
        self.spin_timer_counter = 0
        self.w1 = self.base_sp
        self.w2 = self.base_sp
        self.setWheels(0.0, 0.0)
