from __future__ import division, print_function

import cv2
import numpy as np

from .camera_base import Camera


class CancerousCamera(Camera):
    """Camera class for detecting the parking lot."""

    # The portion of the image we focus on. (y-slice, x-slice).
    REGION_OF_INTEREST = (slice(0, None, None), slice(0, None, None))

    def process_image(self, hsv_image):
        """TODO: What is this supposed to do?"""
        # Crop the image to deal only with whatever is directly in front of us.
        cropped = hsv_image[self.REGION_OF_INTEREST]
        sensitivity = 15
        g_low = np.array([60 - sensitivity, 100, 100])
        g_high = np.array([60 + sensitivity, 255, 255])
        mask = cv2.inRange(cropped, g_low, g_high)

        # TODO: Find a way to publish a vector field avoiding anything
        # that is not green.

        if self.verbose:
            # Mask out only the greens. Everything else will be black.
            masked = cv2.bitwise_and(cropped, cropped, mask=mask)
            cv2.imshow('ParkingLot', cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))
