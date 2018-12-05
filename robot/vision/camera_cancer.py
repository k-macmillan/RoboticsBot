from __future__ import division, print_function

import cv2
import numpy as np
from geometry_msgs.msg import Vector3

from .camera_base import Camera


class CancerousCamera(Camera):
    """Camera class for producing the gradient at the current location.

    This class produces a repulsive gradient away from anything that is not
    green (the lot) or blue (the lot exit).
    """

    # Sensitivity for the green color detection.
    GREEN_SENSITIVITY = 50
    # Sensitivity for the blue color detection.
    BLUE_SENSITIVITY = 40

    def process_image(self, hsv_image):
        """Produce a repulsive gradient away from anything not green/blue."""
        green_low = np.array([60 - self.GREEN_SENSITIVITY, 80, 80])
        green_high = np.array([60 + self.GREEN_SENSITIVITY, 255, 255])

        blue_low = np.array([120 - self.BLUE_SENSITIVITY, 80, 100])
        blue_high = np.array([120 + self.BLUE_SENSITIVITY, 255, 255])

        green_mask = cv2.inRange(hsv_image, green_low, green_high)
        blue_mask = cv2.inRange(hsv_image, blue_low, blue_high)

        # Join the two masks.
        mask = green_mask + blue_mask

        # TODO: Produce a blue mask.

        # TODO: Mask out everything not green or blue.

        # TODO: Find contours in combined mask.

        # TODO: Somehow produce a gradient.

        # TODO: Publish the gradient at the current location.

        if self.verbose:
            # Mask out only the greens. Everything else will be black.
            masked = cv2.bitwise_and(hsv_image, hsv_image, mask=mask)
            cv2.namedWindow('GreenBlueMask', cv2.WINDOW_NORMAL)
            cv2.imshow('GreenBlueMask', cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))
