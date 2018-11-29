#!/usr/bin/env python2
import argparse
from robot import Robot


def parse_args():
    """Parse robot's commandline arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--verbose',
        '-v',
        action='store_true',
        default=False,
        help='Increase robot\'s volume.')
    return parser.parse_args()


def main(args):
    """Main entry point for robot."""
    robot = Robot(verbose=args.verbose)
    robot.start()


if __name__ == "__main__":
    main(parse_args())
