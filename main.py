import argparse
from robot.camera import Camera


def parse_args():
    """Parse robot's commandline arguments."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-index',
                        type=int,
                        default=1,
                        help='index of webcam to use')

    return parser.parse_args()


def main(args):
    """Main entry point for robot."""
    cam = Camera(args.camera_index)
    cam.loop()


if __name__ == "__main__":
    main(parse_args())
