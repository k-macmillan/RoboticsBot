#!/usr/bin/env python2
import sys

from robot.nodes import Joystick, NodeManager


def main():
    """Control the robot with the WASD keys."""
    manager = NodeManager()
    manager.add_node(Joystick(sys.stdin.fileno()))
    manager.spin()


if __name__ == "__main__":
    main()
