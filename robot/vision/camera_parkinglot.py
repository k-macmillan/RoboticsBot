from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import Float32

from .camera_base import Camera


class ParkingLotCamera(Camera):
    """Camera class for stoplight detection."""

    # The portion of the image we focus on. (y-slice, x-slice).
    REGION_OF_INTEREST = (slice(0, None, None), slice(0, None, None))
    # Arbitrary cutoff for determining if an object is detected
    GREEN_CUTOFF = 500

    def process_image(self, hsv_image):
        """Publish a notification of a stoplight is encountered."""
        # Crop the image to deal only with whatever is directly in front of us.
        cropped = hsv_image[self.REGION_OF_INTEREST]
        sensitivity = 15
        g_low = np.array([60 - sensitivity, 100, 100])
        g_high = np.array([60 + sensitivity, 255, 255])
        self.mask = cv2.inRange(cropped, g_low, g_high)

        # We see a stoplight if there are more than some number of red pixels.
        if np.sum(self.mask) / 255 > self.GREEN_CUTOFF:
            print('Found parking lot')
            # msg = String()
            # msg.data = POI['STOPLIGHT']
            # self.publisher.publish(msg)

        if self.verbose:
            # Mask out only the reds. Everything else will be black.
            masked = cv2.bitwise_and(cropped, cropped, mask=self.mask)
            cv2.imshow('Obstacles', cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))
