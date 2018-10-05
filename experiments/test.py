#!/usr/bin/env python2
from node_manager import NodeManager
from nodes import IrSpammer, IrPlotter


def main():
    manager = NodeManager()

    manager.add_node(IrSpammer())
    manager.add_node(IrPlotter(history=200))

    manager.spin()


if __name__ == '__main__':
    main()
