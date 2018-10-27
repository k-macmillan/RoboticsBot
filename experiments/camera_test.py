#!/usr/bin/env python2
import sys
sys.path.append('..')

import rospy as ros
from robot.vision import Camera


def main():
    """Show the NodeManager in action."""
    ros.init_node('Camera', anonymous=True)
    cam = Camera()

    # Do not use ros.spin() because TKinter will freak the fuck out.
    while not ros.is_shutdown():
        cam.tk_update()

    cam.stop()

if __name__ == '__main__':
    main()
