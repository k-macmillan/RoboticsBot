from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import String

from robot.common import POI

from .camera_base import Camera


class StoplightCamera(Camera):
    """Camera class for stoplight detection."""

    # The portion of the image we focus on. (y-slice, x-slice).
    # REGION_OF_INTEREST = (slice(470, 480, None), slice(0, None, None))
    # How much do we blur the image
    BLUR_KERNEL = (5, 5)
    # How many red pixels count as a stoplight. Lol.
    SENSITIVITY = 50

    def process_image(self, hsv_image):
        """Publish a notification of a stoplight is encountered."""
        # Crop the image to deal only with whatever is directly in front of us.
        # cropped = hsv_image[self.REGION_OF_INTEREST]
        cropped = hsv_image

        # Mask out everything but white.
        w_low = np.array([0, 0, 255 - self.SENSITIVITY])
        w_high = np.array([255, self.SENSITIVITY, 255])

        # Mask out everything but black.
        blk_low = np.array([0, 0, 0])
        blk_high = np.array([180, 255, 90])

        w_mask = cv2.inRange(cropped, w_low, w_high)
        blk_mask = cv2.inRange(cropped, blk_low, blk_high)

        # Join the two masks.
        # w_img = cv2.bitwise_and(cropped, cropped, mask=w_mask)
        mask = w_mask + blk_mask

        # We see a stoplight if there are more than some number of red pixels.
        # if np.sum(mask) / 255 > self.RED_SENSITIVITY:
        #     msg = String()
        #     msg.data = POI['STOPLIGHT']
        #     self.publisher.publish(msg)

        if self.verbose:
            # Mask out only the reds. Everything else will be black.
            # masked = cv2.bitwise_and(cropped, cropped, mask=mask)
            cv2.imshow('Stoplight-masked2', mask)
