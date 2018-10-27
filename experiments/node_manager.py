from __future__ import print_function

import multiprocessing

import rospy as ros


class Node(multiprocessing.Process):
    """A base Node object.

    Subclass multiprocessing.Process because each Node must be initialized and
    spun in its own process.
    """

    def __init__(self, name):
        """Creates and runs a ROS node with the given name.

        NOTE: This method gets run in the main process.
        """
        super(Node, self).__init__()
        self.__name = name

    def run(self):
        """Runs the ROS Node. If a derived class overrides this method, the
        derived class *must* call its parent's blocking start() method.

        NOTE: This method gets run in the created process.
        """
        ros.init_node(self.__name, anonymous=True, disable_signals=False)
        # Set this node's shutdown signal handler.
        ros.on_shutdown(self.stop)
        # Immediately begin spinning this Node.
        ros.spin()

    def stop(self):
        """Signal handler to stop the ROS Node.

        Should be implemented by a derived class if the derived class requires
        a notification that it's being terminated.
        """
        print('Terminating Node: ', self.__name)


class NodeManager(object):
    """A class to manage multiple ROS nodes, each in their own process.

    NOTE: rospy.init_node requires each node to be created in a different process.
          This is directly at odds at having a single Python entry point for
          bringing a collection of nodes online.
          Note however, that ROS launchfiles solve this problem, but I have no
          desire to learn yet another new thing.
    """
    def __init__(self):
        """Creates a NodeManager for running ROS nodes."""
        self.jobs = []

    def add_node(self, node):
        """Add a node of the given type to the NodeManager.

        :param node: An instance of some subclass of Node.
        """
        # Add a the process to the list of jobs.
        self.jobs.append(node)

    def spin(self):
        """Run each node in its own process, and wait for them to finish."""
        for job in self.jobs:
            job.start()

        for job in self.jobs:
            try:
                job.join()
            except KeyboardInterrupt:
                break

        for job in self.jobs:
            job.terminate()
