from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import String

from robot.common import POI

from .camera_base import Camera


class ObstacleCamera(Camera):
    """Camera class for stoplight detection."""

    # The portion of the image we focus on. (y-slice, x-slice).
    REGION_OF_INTEREST = (slice(0, None, None), slice(0, None, None))
    # How much do we blur the image
    BLUR_KERNEL = (5, 5)
    # Arbitrary cutoff for determining if an object is detected
    YELLOW_CUTOFF = 500

    def process_image(self, hsv_image):
        """Publish a notification of a stoplight is encountered."""
        # Crop the image to deal only with whatever is directly in front of us.
        cropped = hsv_image[self.REGION_OF_INTEREST]
        # Image should be blurred coming out of the camera, all cameras require
        # a blur to best catch POIs.
        blurred = cv2.GaussianBlur(cropped, self.BLUR_KERNEL, 0)
        sensitivity = 5
        y_low = np.array([25 - sensitivity, 100, 100])
        y_high = np.array([25 + sensitivity, 255, 255])
        mask = cv2.inRange(blurred, y_low, y_high)

        # We see a stoplight if there are more than some number of red pixels.
        if np.sum(mask) / 255 > self.YELLOW_CUTOFF:
            pass
            # msg = String()
            # msg.data = POI['STOPLIGHT']
            # self.publisher.publish(msg)

        if self.verbose:
            # Mask out only the reds. Everything else will be black.
            masked = cv2.bitwise_and(blurred, blurred, mask=mask)
            cv2.imshow('Obstacles',
                       cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))
