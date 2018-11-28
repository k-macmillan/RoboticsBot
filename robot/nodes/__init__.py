"""A collection of ROS nodes for the robot."""

from .node_manager import Node, NodeManager
from .ir_sensor import IrPlotter, IrSpammer
from .brain import Brain
from .wheels import Wheels
