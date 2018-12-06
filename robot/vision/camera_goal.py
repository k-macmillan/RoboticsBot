from __future__ import division, print_function

import cv2
import numpy as np
from std_msgs.msg import Float32, String

from robot.common import POI

from .camera_base import Camera


class GoalCamera(Camera):
    """Camera class for detecting the goal."""

    # Sensitivity for the blue color detection.
    BLUE_SENSITIVITY = 40
    # The threshold value.
    THRESH_VALUE = 70
    # The threshold maximum value.
    THRESH_MAX = 255
    # The minimum area of a contour required to be considered the goal.
    MIN_GOAL_AREA = 400

    def __init__(self, error_pub, poi_pub, verbose=False):
        """Construct a GoalCamera.

        Overrides the default __init__ defined by the parent class.

        :param error_pub: The error signal publisher.
        :type error_pub: rospy.publisher
        :param poi_pub: The Point Of Interest publisher.
        :type poi_pub: rospy.publisher
        :param verbose: If we should spam stuff, defaults to False
        :type verbose: bool, optional
        """
        # Don't use the self.publisher attribute for clarity. We need multiple
        # publishers.
        super(GoalCamera, self).__init__(None, verbose=verbose)

        self.error_pub = error_pub
        self.poi_pub = poi_pub

    def process_image(self, hsv_image):
        """Publish left/right relative position of the goal.

        Publish a float between -1 and 1 to indicate relative position of the
        goal to the center of the frame. If the goal is not visible, publish an
        absurd value.
        """
        # These values are appropriate at max brightness.
        blue_low = np.array([120 - self.BLUE_SENSITIVITY, 40, 80])
        blue_high = np.array([120 + self.BLUE_SENSITIVITY, 255, 255])

        blue_mask = cv2.inRange(hsv_image, blue_low, blue_high)
        # Mask out only the greens. Everything else will be black.
        masked = cv2.bitwise_and(hsv_image, hsv_image, mask=blue_mask)

        if self.verbose:
            cv2.namedWindow('GoalCamera-mask', cv2.WINDOW_NORMAL)
            cv2.imshow('GoalCamera-mask',
                       cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))

        # Convert to grayscale.
        grayscale = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        # Threshold the grays.
        _, thresh = cv2.threshold(grayscale, self.THRESH_VALUE,
                                  self.THRESH_MAX, cv2.THRESH_BINARY)
        if self.verbose:
            cv2.namedWindow('GoalCamera-thresh', cv2.WINDOW_NORMAL)
            cv2.imshow('GoalCamera-thresh', thresh)

        _, contours, _ = cv2.findContours(thresh, 1, cv2.CHAIN_APPROX_SIMPLE)

        # If we find any contours, find the biggest and call that the goal.
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)

            M = cv2.moments(max_contour)
            # If the contour area is bigger than some threshold, try to find
            # its centroid, if possible.
            if area > self.MIN_GOAL_AREA and M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                # cy = int(M['m01'] / M['m00'])

                # Normalize the error so that it's -1.0 to 1.0, with 0.0 being
                # an indication that the goal centroid is in the exact center
                # of the frame.
                error = 0.0
                mid = hsv_image.shape[1] / 2
                if cx <= mid:
                    error = (cx - mid) / mid
                else:
                    error = -(mid - cx) / mid

                msg = Float32()
                msg.data = error
                self.error_pub.publish(msg)

                msg = String()
                msg.data = POI['EXIT_LOT']
                self.poi_pub.publish(msg)
                return

        msg = Float32()
        msg.data = 0.0
        self.error_pub.publish(msg)

        msg = String()
        msg.data = POI['NO_EXIT_LOT']
        self.poi_pub.publish(msg)
