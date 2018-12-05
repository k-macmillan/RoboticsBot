from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import Float32

from .camera_base import Camera


class GoalCamera(Camera):
    """Camera class for detecting the goal."""

    # Sensitivity for the blue color detection.
    BLUE_SENSITIVITY = 40
    # The threshold value.
    THRESH_VALUE = 70
    # The threshold maximum value.
    THRESH_MAX = 255

    def process_image(self, hsv_image):
        """Publish left/right relative position of the goal.

        Publish a float between -1 and 1 to indicate relative position of the
        goal to the center of the frame. If the goal is not visible, publish an
        absurd value.
        """

        # These values are appropriate ~50 Lux.
        blue_low = np.array([120 - self.BLUE_SENSITIVITY, 80, 100])
        blue_high = np.array([120 + self.BLUE_SENSITIVITY, 255, 255])

        blue_mask = cv2.inRange(hsv_image, blue_low, blue_high)
        # Mask out only the greens. Everything else will be black.
        masked = cv2.bitwise_and(hsv_image, hsv_image, mask=blue_mask)

        if self.verbose:
            cv2.namedWindow('GoalCamera-mask', cv2.WINDOW_NORMAL)
            cv2.imshow('GoalCamera-mask',
                       cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))

        # Convert to grayscale.
        # TODO: BGR to grayscale?! This is an HSV image...
        grayscale = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        # Threshold the grays.
        _, thresh = cv2.threshold(grayscale, self.THRESH_VALUE,
                                  self.THRESH_MAX, cv2.THRESH_BINARY)
        if self.verbose:
            cv2.namedWindow('GoalCamera-thresh', cv2.WINDOW_NORMAL)
            cv2.imshow('GoalCamera-thresh', thresh)

        _, contours, _ = cv2.findContours(thresh, 1, cv2.CHAIN_APPROX_SIMPLE)

        fraction = 1000.0
        if contours:
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                image_center = hsv_image.shape[1] / 2
                if cx <= image_center:
                    fraction = (cx - image_center) / image_center
                else:
                    fraction = -(image_center - cx) / image_center

                if self.verbose:
                    print('GoalCamera: goal centroid: ({}, {})'.format(cx, cy))

        if self.verbose:
            print('GoalCamera: control signal:', fraction)

        msg = Float32()
        msg.data = fraction
        self.publisher.publish(msg)
