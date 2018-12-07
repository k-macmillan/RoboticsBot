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
    parser.add_argument(
        'target',
        type=int,
        default=0,
        help='The target graph node. Default is 0'
    )
    return parser.parse_args()


def main(args):
    """Main entry point for robot."""
    robot = Robot(target=args.target, verbose=args.verbose)
    robot.start()


if __name__ == "__main__":
    main(parse_args())
