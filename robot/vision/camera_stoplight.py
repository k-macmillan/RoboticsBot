from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import String

from robot.common import POI

from .camera_base import Camera


class StoplightCamera(Camera):
    """Camera class for stoplight detection."""

    # The portion of the image we focus on. (y-slice, x-slice).
    REGION_OF_INTEREST = (slice(470, 480, None), slice(0, None, None))
    # How much do we blur the image
    BLUR_KERNEL = (5, 5)
    SENSITIVITY = 50
    # How many red pixels count as a stoplight. Lol.
    STOP_THRESHOLD = 1000

    def process_image(self, hsv_image):
        """ Publish a notification of a stoplight is encountered.
            inspiration: https://stackoverflow.com/a/25401596
        """
        # Crop the image to deal only with whatever is directly in front of us.
        hsv_image = hsv_image[self.REGION_OF_INTEREST].copy()

        # Mask out everything but white.
        w_low = np.array([0, 0, 255 - self.SENSITIVITY])
        w_high = np.array([255, self.SENSITIVITY, 255])

        # Mask out everything but black.
        blk_low = np.array([0, 0, 0])
        blk_high = np.array([180, 255, 200])

        w_mask = cv2.inRange(hsv_image, w_low, w_high)
        w_mask = cv2.erode(
            w_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)),
            iterations=1)
        w_mask = cv2.dilate(
            w_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1)
        blk_mask = cv2.inRange(hsv_image, blk_low, blk_high)
        blk_mask = cv2.erode(
            blk_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)),
            iterations=1)
        blk_mask = cv2.dilate(
            blk_mask,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
            iterations=1)

        if self.verbose:
            cv2.namedWindow('Stoplight W Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Stoplight W Mask', w_mask)

            cv2.namedWindow('Stoplight BLK Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Stoplight BLK Mask', blk_mask)

        # Join the two masks.
        mask = w_mask + blk_mask
        # Swap 0 and 255 values...
        mask[mask == 0] = 1
        mask[mask >= 255] = 0
        mask[mask == 1] = 255

        # We see a stoplight if there are more than some number of red pixels.
        if np.sum(mask) / 255 > self.STOP_THRESHOLD:
            msg = String()
            msg.data = POI['STOPLIGHT']
            self.publisher.publish(msg)

        if self.verbose:
            # Mask out only the reds. Everything else will be black.
            # masked = cv2.bitwise_and(cropped, cropped, mask=mask)
            cv2.namedWindow('Stoplight W+B Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Stoplight W+B Mask', mask)
