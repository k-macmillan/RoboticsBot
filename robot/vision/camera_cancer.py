from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import String

from robot.common import POI

from .camera_base import Camera


class CancerousCamera(Camera):
    """Publish obstacle obstruction information."""

    # The region where the obstacle would be directly in front of us.
    REGION_OF_INTEREST = (slice(450, 460, None), slice(0, None, None))
    # Sensitivity for the green color detection.
    GREEN_SENSITIVITY = 25
    # Sensitivity for the blue color detection.
    BLUE_SENSITIVITY = 40
    # How many pixels of obstacle should count as an obstruction.
    OBSTRUCTION_TOLERANCE = 100

    def process_image(self, hsv_image):
        """Determine if there is an obstacle directly in front of the robot."""
        # hsv_image = hsv_image[self.REGION_OF_INTEREST].copy()

        # I don't know what light intensity these work for.
        # green_low = np.array([60 - self.GREEN_SENSITIVITY, 80, 80])
        # green_high = np.array([60 + self.GREEN_SENSITIVITY, 255, 255])

        # blue_low = np.array([120 - self.BLUE_SENSITIVITY, 80, 100])
        # blue_high = np.array([120 + self.BLUE_SENSITIVITY, 255, 255])

        # These thresholds work for a light intensity of ~90lx on an S8
        green_low = np.array([60 - self.GREEN_SENSITIVITY, 40, 80])
        green_high = np.array([60 + self.GREEN_SENSITIVITY, 255, 255])

        blue_low = np.array([120 - self.BLUE_SENSITIVITY, 40, 80])
        blue_high = np.array([120 + self.BLUE_SENSITIVITY, 255, 255])

        green_mask = cv2.inRange(hsv_image, green_low, green_high)
        blue_mask = cv2.inRange(hsv_image, blue_low, blue_high)

        if self.verbose:
            cv2.namedWindow('Obstacle G Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle G Mask', green_mask)
            cv2.namedWindow('Obstacle B Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle B Mask', blue_mask)

        # Join the two masks.
        mask = green_mask + blue_mask

        # Swap 0 and 255 values in the combined mask.
        mask[mask == 0] = 1
        mask[mask == 255] = 0
        mask[mask == 1] = 255

        msg = String()
        if np.sum(mask) / 255 >= self.OBSTRUCTION_TOLERANCE:
            msg.data = POI['OBSTACLE']
        else:
            msg.data = POI['NO_OBSTACLE']
        self.publisher.publish(msg)

        if self.verbose:
            cv2.namedWindow('Obstacle G+B Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle G+B Mask', mask)
