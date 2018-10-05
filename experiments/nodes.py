#!/usr/bin/env python2
from __future__ import print_function

import multiprocessing as mp
from collections import deque

import matplotlib.pyplot as plt
import rospy as ros
from std_msgs.msg import Int32

from node_manager import Node


class IrSpammer(Node):
    """A ROS Node to spam IR data to the console. For testing purposes only."""
    def __init__(self):
        """A basically useless ROS Node to spam the console with IR sensor data."""
        super(IrSpammer, self).__init__(name='IrSpammer')
        ros.Subscriber('/geekbot/ir_cm', Int32, self.callback)

    def callback(self, msg):
        """Receives an IR sensor reading and prints it to the screen."""
        print('IR:', msg.data)


class IrPlotter(Node):
    """A ROS Node to live plot the data streamed from the Geekbot IR sensor.

    The GUI plot window *should* be closed before terminating the Node, but the
    worst that will happen if you don't is that a few exceptions will be raised
    in non-deterministic places.
    """
    def __init__(self, history=800):
        """Creates an IrPlotter ROS Node to live plot IR sensor data."""
        super(IrPlotter, self).__init__(name='IrPlotter')
        ros.Subscriber('/geekbot/ir_cm', Int32, self.callback)
        self.queue = mp.Queue()
        self.child = None
        self.history = history

    def callback(self, msg):
        """Receives IR distance data from the Geekbot IR sensor"""
        # Pass the received value to the plotter child process.
        self.queue.put(msg.data, block=False)

    def plotter(self):
        """The body of the plotter child process to live plot a time series
        with matplotlib.
        """
        ys = deque([], maxlen=self.history)
        fig, ax = plt.subplots()
        plt.ion()
        plot, = ax.plot([])
        plt.title('Robot IR Sensor Readings')

        while True:
            # If the plotting window is closed, exit.
            if not plt.fignum_exists(fig.number):
                break
            # Get the next sensor reading from the queue.
            ys.append(self.queue.get(block=True))
            # Need to set both x and y data in the time series.
            plot.set_data(range(len(ys)), ys)

            # Fit the plot window to the data.
            ax.autoscale()
            ax.relim()

            plt.pause(0.0001)

    def run(self):
        """Runs this node in a new process."""
        # Create the child [process] here, because children must be created by
        # their parents, and not by the NodeManager's process.
        self.child = mp.Process(target=self.plotter)
        self.child.daemon = True
        self.child.start()

        # Derived classes *must* call the parent's run()
        super(IrPlotter, self).run()

    def stop(self):
        """Shutdown signal handler to clean up after ourselves"""
        if self.child.is_alive():
            self.child.terminate()
