from __future__ import division, print_function

import itertools
import random

import rospy as ros
from std_msgs.msg import Float32, Float32MultiArray, String, UInt8

from robot.common import *
from robot.nodes import DriveLine, Node


class Brain(Node):
    """A ROS Node to handle the brain of our robot."""

    def __init__(self, node=0, verbose=False):
        """Initialize the Brain node.

        :param verbose: How passionate should the Brain be?, defaults to False
        :param verbose: bool, optional
        """
        super(Brain, self).__init__(name='Brain')
        self.verbose = verbose
        self.state = State.ON_PATH
        self.turn_dir = 1
        self.node_list = self.pairwise(GRAPH_PATH[node])
        self.node_slice = None
        self.rl_count = 0
        self.spin_timer_counter = 0
        self.node0 = False
        self.done = False
        self.target = node
        self.rotation = 1

        # Timer vars
        self.state_timer = None
        self.spin_timer = None
        self.rl_timer = None
        self.node_timer = None
        self.rotate_timer = None
        self.node0_timer = None

        # POI
        self.stoplight_POI = False
        self.obstacle_POI = False
        self.goal_POI = False
        self.node_POI = False

        # Errors
        self.path_error = 0.0
        self.goal_error = 0.0
        self.node_error = 0.0

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
        ros.Subscriber(TOPIC['NODE_CENTROID'], Float32, self.topicNode)
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
        # We require a bootstrap.
        if self.state_timer is None:
            self.stateTimer()
        self.goal_error = msg.data

    def topicNode(self, msg):
        # We require a bootstrap.
        if self.state_timer is None:
            self.stateTimer()
        self.node_error = msg.data

    def topicPOI(self, msg):
        """Handle a Point of Interest notification.

        This determines the robot's current state.

        :param msg: The point of interest notification.
        :type msg: std_msgs.msg.String
        """
        if msg.data == POI['STOPLIGHT'] and self.state == State.ON_PATH:
            self.stoplight_POI = True
        elif msg.data == POI['OBSTACLE']:
            # print('OBSTRUCTED')
            self.obstacle_POI = True
        elif msg.data == POI['NO_OBSTACLE']:
            # print('\tNO OBSTRUCTION')
            self.obstacle_POI = False
        elif msg.data == POI['EXIT_LOT']:
            self.goal_POI = True
        elif msg.data == POI['NO_EXIT_LOT']:
            self.goal_POI = False
        elif msg.data == POI['GRAPH_NODE']:
            self.node_POI = True

    def stateHandler(self, event):
        # Path
        if self.state == State.ON_PATH:
            self.pathState()
        elif self.state == State.STOPPING:
            self.stoppingState()
        elif self.state == State.STOPPED:
            self.stoppedState()
        # Parking Lot
        elif self.state == State.CANCER:
            self.base_sp = 7.0
            self.cancerState()
        elif self.state == State.SPIN:
            self.spinState()
        elif self.state == State.TURN:
            self.turnState()
        elif self.state == State.MTG:
            self.mtgState()
        # Graph
        elif self.state == State.GRAPH:
            self.base_sp = 8.0
            self.graphState()
        elif self.state == State.ORIENTING:
            self.orientingState()
        elif self.state == State.G_ON_PATH:
            self.graphOnPathState()
        elif self.state == State.NODE_STOPPING:
            self.nodeStoppingState()
        elif self.state == State.NODE_STOPPED:
            self.nodeStoppedState()
        elif self.state == State.ROTATE_LEFT:
            self.rotateLeftState()
        elif self.state == State.ROTATE_RIGHT:
            self.rotateRightState()
        elif self.state == State.FORWARD:
            self.forwardState()
        elif self.state == State.END:
            self.endState()

    # Path section

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

    # Parking lot section

    def cancerState(self):
        if self.obstacle_POI:
            self.rotation = -1 * self.rotation
            self.transition(State.SPIN)
        elif self.goal_POI:
            self.w1, self.w2 = 5.0, 5.0
            self.transition(State.MTG)
        else:
            self.setWheels(self.base_sp, self.base_sp)

    def spinState(self):
        self.setWheels(self.base_sp * self.rotation,
                       -self.base_sp * self.rotation)
        self.startSpinTimer()

    def turnState(self):
        if self.obstacle_POI:
            self.setWheels(self.base_sp * self.turn_dir,
                           -self.base_sp * self.turn_dir)
        else:
            self.transition(State.CANCER)

    def mtgState(self):
        if self.goal_POI:
            self.w1, self.w2 = self.DL.calcWheelSpeeds(self.w1,
                                                       self.w2,
                                                       self.goal_error)
            self.setWheels(self.w1, self.w2)
        else:
            self.setWheels(0.0, 0.0)
            self.transition(State.GRAPH)

    # Graph section

    def graphState(self):
        # I am slightly worried about losing goal vision.
        self.last_centroid = self.path_error
        self.transition(State.ORIENTING)

    def orientingState(self):
        # Keep track of last lane centroid
        if self.path_error != self.last_centroid:
            self.setWheels(self.base_sp, -self.base_sp)
        else:
            self.transition(State.G_ON_PATH)

    def graphOnPathState(self):
        if self.node_POI:
            self.transition(State.NODE_STOPPING)
        else:
            self.w1, self.w2 = self.DL.calcWheelSpeeds(self.w1,
                                                       self.w2,
                                                       self.path_error)
            self.setWheels(self.w1, self.w2)

    def nodeStoppingState(self):
        if self.node_timer is None:
            self.nodeTimer(2.0)

    def nodeStoppedState(self):
        if not self.node0:
            self.setWheels(self.base_sp, -self.base_sp)
            self.node0Timer()
            # Avoid state transition until after timer finishes.
            return
        else:
            self.node_slice = next(self.node_list, None)
            print('node slice:', self.node_slice)

        if self.node_slice is None:
            self.transition(State.END)
        else:
            self.transition(State.ROTATE_LEFT)

    def rotateLeftState(self):
        if self.node_slice in LEFT_TURN:
            if self.rotate_timer is None:
                print('Rotate left.')
                self.setWheels(-self.base_sp, self.base_sp)
                if self.node_slice[1] == 5:
                    self.rotateTimer(1.5)
                else:
                    self.rotateTimer(0.7)
        else:
            self.transition(State.ROTATE_RIGHT)

    def rotateRightState(self):
        if self.node_slice in RIGHT_TURN:
            if self.rotate_timer is None:
                print('Rotate right.')
                self.setWheels(self.base_sp, -self.base_sp)
                self.rotateTimer(1.0)
        else:
            self.transition(State.FORWARD)

    def forwardState(self):
        if self.node_POI:
            self.transition(State.NODE_STOPPING)
        else:
            print('node error:', self.node_error)
            self.w1, self.w2 = self.DL.calcWheelSpeeds(self.w1,
                                                       self.w2,
                                                       self.node_error)
            self.setWheels(self.w1, self.w2)

    def endState(self):
        self.setWheels(0.0, 0.0)
        if not self.done:
            self.done = True
            print('VICTORY')

    # Helper functions

    def setWheels(self, w1=None, w2=None):
        if w1 is None or w2 is None:
            w1 = self.w1
            w2 = self.w2
        wheels = Float32MultiArray()
        wheels.data = [w1 + 0.5, w2]
        self.wheel_speeds.publish(wheels)

    def printError(self, msg):
        for i in range(20):
            print(msg)

    @staticmethod
    def pairwise(iterable):
        """s -> (s0,s1), (s{1,s2), (s2, s3), ..."""
        a, b = itertools.tee(iterable)
        next(b, None)
        return itertools.izip(a, b)

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
                ros.Duration(secs=1.3), self.timerRLShutdown)

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

    def nodeTimer(self, time):
        if self.node_timer is None:
            print('Creating Node timer')
            self.node_timer = ros.Timer(
                ros.Duration(secs=time), self.timerNodeShutdown)

    def timerNodeShutdown(self, event):
        self.node_timer.shutdown()
        self.node_timer = None
        self.transition(State.NODE_STOPPED)
        self.node_POI = False
        self.setWheels(0.0, 0.0)

    def rotateTimer(self, time):
        if self.rotate_timer is None:
            print('Creating Rotate timer')
            self.rotate_timer = ros.Timer(
                ros.Duration(secs=time), self.timerRotateShutdown)

    def timerRotateShutdown(self, event):
        self.rotate_timer.shutdown()
        self.rotate_timer = None
        self.transition(State.FORWARD)
        self.setWheels(0.0, 0.0)

    def node0Timer(self):
        if self.node0_timer is None:
            print('Creating Node ZERO timer')
            self.node0_timer = ros.Timer(
                ros.Duration(secs=0.5), self.timerNode0Shutdown)

    def timerNode0Shutdown(self, event):
        print('Shutting down Node ZERO timer')
        self.node0_timer.shutdown()
        self.node0_timer = None
        self.node0 = True
        self.setWheels(0.0, 0.0)
