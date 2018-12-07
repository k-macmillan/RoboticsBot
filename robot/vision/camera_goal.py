from __future__ import division, print_function

from collections import deque
import cv2
from std_msgs.msg import Float32, String

from robot.common import POI

from .camera_base import Camera
from .mask import mask_image


class GoalCamera(Camera):
    """Camera class for detecting the goal."""

    # Sensitivity for the blue color detection.
    BLUE_SENSITIVITY = 10
    # The threshold value.
    THRESH_VALUE = 70
    # The threshold maximum value.
    THRESH_MAX = 255
    # The minimum area of a contour required to be considered the goal.
    MIN_GOAL_AREA = 10000

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
        self.detections = deque([0]*10, maxlen=10)
        # self.counter = 0

    def process_image(self, hsv_image):
        """Publish left/right relative position of the goal.

        Publish a float between -1 and 1 to indicate relative position of the
        goal to the center of the frame. If the goal is not visible, publish an
        absurd value.
        """
        # self.counter += 1
        # if self.counter % 10 == 0:
        #     print('counter:', self.counter)
        # These values are appropriate at max brightness.
        blue_mask = mask_image(hsv_image, (100, 20, 80), (130, 255, 255))

        if self.verbose:
            cv2.namedWindow('Goal B Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Goal B Mask', blue_mask)

        _, contours, _ = cv2.findContours(blue_mask, 1, cv2.CHAIN_APPROX_SIMPLE)

        goal_in_sight = False
        error = Float32()
        error.data = 0.0
        poi = String()
        poi.data = POI['NO_EXIT_LOT']

        # If we find any contours, find the biggest and call that the goal.
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            # print('goal area:', area)

            M = cv2.moments(max_contour)
            # If the contour area is bigger than some threshold, try to find
            # its centroid, if possible.
            if area > self.MIN_GOAL_AREA and M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                # cy = int(M['m01'] / M['m00'])

                # Normalize the error so that it's -1.0 to 1.0, with 0.0 being
                # an indication that the goal centroid is in the exact center
                # of the frame.
                signal = 0.0
                mid = hsv_image.shape[1] / 2
                if cx <= mid:
                    signal = (cx - mid) / mid
                else:
                    signal = -(mid - cx) / mid

                error.data = signal
                goal_in_sight = True

        self.detections.appendleft(goal_in_sight)
        if sum(self.detections) != 0:
            poi.data = POI['EXIT_LOT']
        else:
            poi.data = POI['NO_EXIT_LOT']

        self.error_pub.publish(error)
        self.poi_pub.publish(poi)
