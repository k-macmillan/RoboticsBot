#!/usr/bin/env python2
from __future__ import print_function

import multiprocessing as mp
from collections import deque

import matplotlib.pyplot as plt
import rospy as ros
from std_msgs.msg import Int32


class IrPlotter(object):
    """Live plots the data streamed from the Geekbot IR sensor."""
    def __init__(self, history=800):
        """Registers IrPlotter node with ROS master."""
        ros.init_node('IrPlotter', anonymous=True, disable_signals=True)
        ros.on_shutdown(self.stop)
        self.subscriber = ros.Subscriber('/geekbot/ir_cm', Int32, self.callback)
        self.queue = mp.Queue()
        self.history = history
        self.job = mp.Process(target=self.plotter)

    def callback(self, msg):
        """Receives IR distance data from the Geekbot IR sensor"""
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

    def start(self):
        """Runs this node"""
        self.job.start()
        print('Starting plotter process with PID', self.job.pid)

    def stop(self):
        """Shutdown signal handler to clean up after ourselves"""
        print('Terminating plotter process with PID', self.job.pid)
        self.job.terminate()


if __name__ == '__main__':
    plotter = IrPlotter()
    plotter.start()

    ros.spin()
