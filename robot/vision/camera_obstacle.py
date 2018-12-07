from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import String

from robot.common import POI

from .camera_base import Camera


class ObstacleCamera(Camera):
    """Publish obstacle obstruction information."""

    # The region where the obstacle would be directly in front of us.
    REGION_OF_INTEREST = (slice(450, 460, None), slice(0, None, None))
    # Sensitivity for the green color detection.
    GREEN_SENSITIVITY = 20
    # Sensitivity for the blue color detection.
    BLUE_SENSITIVITY = 10
    # How many pixels of obstacle should count as an obstruction.
    OBSTRUCTION_TOLERANCE = 200

    def process_image(self, hsv_image):
        """Determine if there is an obstacle directly in front of the robot."""
        hsv_image = hsv_image[self.REGION_OF_INTEREST]

        # These thresholds work for a light intensity of ~90lx on an S8
        green_low = np.array([40, 25, 50])
        green_high = np.array([80, 255, 255])

        blue_low = np.array([100, 80, 100])
        blue_high = np.array([130, 255, 255])

        yellow_low = np.array([10, 0, 0])
        yellow_high = np.array([50, 255, 255])

        green_mask = cv2.inRange(hsv_image, green_low, green_high)
        green_mask = cv2.erode(
            green_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)),
            iterations=1)
        green_mask = cv2.dilate(
            green_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1)

        blue_mask = cv2.inRange(hsv_image, blue_low, blue_high)
        blue_mask = cv2.erode(
            blue_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)),
            iterations=1)
        blue_mask = cv2.dilate(
            blue_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1)

        yellow_mask = cv2.inRange(hsv_image, yellow_low, yellow_high)
        yellow_mask = cv2.erode(
            yellow_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)),
            iterations=1)
        yellow_mask = cv2.dilate(
            yellow_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1)

        if self.verbose:
            cv2.namedWindow('Obstacle G Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle G Mask', green_mask)
            cv2.namedWindow('Obstacle B Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle B Mask', blue_mask)
            cv2.namedWindow('Obstacle Y Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle Y Mask', yellow_mask)

        # Join the two masks. This filters everything out but the "good" stuff
        mask = green_mask + blue_mask

        # Swap 0 and 255 values in the combined mask.
        mask[mask == 0] = 1
        mask[mask >= 255] = 0
        mask[mask == 1] = 255

        mask = mask + yellow_mask
        mask[mask >= 255] = 255

        msg = String()
        if np.sum(mask) / 255 >= self.OBSTRUCTION_TOLERANCE:
            msg.data = POI['OBSTACLE']
        else:
            msg.data = POI['NO_OBSTACLE']
        self.publisher.publish(msg)

        if self.verbose:
            cv2.namedWindow('Obstacle G+B-Y Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Obstacle G+B-Y Mask', mask)
