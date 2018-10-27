#!/usr/bin/env python2
import sys
sys.path.append('..')

from robot.nodes import IrPlotter, IrSpammer, NodeManager


def main():
    """Show the NodeManager in action."""
    manager = NodeManager()

    manager.add_node(IrSpammer())
    manager.add_node(IrPlotter(history=200))

    manager.spin()


if __name__ == '__main__':
    main()
