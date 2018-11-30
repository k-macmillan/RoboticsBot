from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import Float32

from .camera_base import Camera


class LaneCamera(Camera):
    """Camera class for lane following."""

    # The portion of the image we focus on. (y-slice, x-slice).
    REGION_OF_INTEREST = (slice(470, 480, None), slice(0, None, None))
    # How much do we blur the image
    BLUR_KERNEL = (5, 5)
    # The threshold value.
    THRESH_VALUE = 70
    # The threshold maximum value.
    THRESH_MAX = 255
    # The white mask sensitivity.
    WHITE_SENSITIVITY = 15

    def process_image(self, hsv_image):
        """Implement lane detection and publishes the lane centroid."""
        # Crop the image to deal only with whatever is directly in front of us.
        cropped = hsv_image[self.REGION_OF_INTEREST]

        # Mask out everything but white.
        white_low = np.array([0, 0, 255 - self.WHITE_SENSITIVITY])
        white_high = np.array([255, self.WHITE_SENSITIVITY, 255])

        mask = cv2.inRange(cropped, white_low, white_high)
        masked = cv2.bitwise_and(cropped, cropped, mask=mask)

        if self.verbose:
            cv2.namedWindow('LaneCamera-mask', cv2.WINDOW_NORMAL)
            cv2.imshow('LaneCamera-mask',
                       cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))

        # Convert to grayscale.
        grayscale = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        # Blur the image before finding contours.
        blurred = cv2.GaussianBlur(grayscale, self.BLUR_KERNEL, 0)
        # Threshold the grays.
        _, thresh = cv2.threshold(blurred, self.THRESH_VALUE, self.THRESH_MAX,
                                  cv2.THRESH_BINARY)

        if self.verbose:
            cv2.namedWindow('LaneCamera-thresh', cv2.WINDOW_NORMAL)
            cv2.imshow('LaneCamera-thresh', thresh)

        # Find contours in the ROI.
        _, contours, _ = cv2.findContours(thresh, 1, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the biggest contour.
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)

            # Avoid division by zero...
            if M['m00'] != 0:
                # Find the centroid of the biggest contour.
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Image center => 0.0, left border => -1.0, right border => 1.0
                image_center = cropped.shape[1] / 2
                fraction = 0.0
                if cx <= image_center:
                    fraction = (cx - image_center) / image_center
                else:
                    fraction = -(image_center - cx) / image_center

                if self.verbose:
                    print('LaneCamera: contour centroid: ({}, {})'.format(
                        cx, cy))
                    print('LaneCamera: control signal:', fraction)

                msg = Float32()
                msg.data = fraction
                self.publisher.publish(msg)

                # Draw the contours in green.
                cv2.drawContours(cropped, contours, -1, (0, 255, 0), 2)
                # Draw the biggest contour centroid in red.
                cv2.circle(
                    cropped, (cx, cy), 10, (255, 0, 0), lineType=cv2.LINE_AA)
        elif self.verbose:
            print('LaneCamera: failed to find contours.')

        if self.verbose:
            cv2.namedWindow('LaneCamera-centroid', cv2.WINDOW_NORMAL)
            cv2.imshow('LaneCamera-centroid',
                       cv2.cvtColor(cropped, cv2.COLOR_HSV2BGR))
