from __future__ import division, print_function

import cv2
from std_msgs.msg import Float32, String

from robot.common import POI

from .camera_base import Camera
from .mask import mask_image


class NodeCamera(Camera):
    """Camera class for detecting the goal."""

    # The minimum area of a contour required to be considered the goal.
    MIN_GOAL_AREA = 20000

    def __init__(self, error_pub, poi_pub, verbose=False):
        """Construct a NodeCamera.

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
        super(NodeCamera, self).__init__(None, verbose=verbose)

        self.error_pub = error_pub
        self.poi_pub = poi_pub

    def process_image(self, hsv_image):
        """Publish left/right relative position of the node.

        Publish a float between -1 and 1 to indicate relative position of the
        node to the center of the frame. If the node is not visible, publish
        a 0.0.
        """
        # These values are appropriate at max brightness.
        purple_mask = mask_image(hsv_image, (100, 80, 100), (130, 255, 255))

        if self.verbose:
            cv2.namedWindow('Node P Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Node P Mask', purple_mask)

        _, contours, _ = cv2.findContours(purple_mask, 1, cv2.CHAIN_APPROX_SIMPLE)

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
                msg.data = POI['GRAPH_NODE']
                self.poi_pub.publish(msg)
                return

        msg = Float32()
        msg.data = 0.0
        self.error_pub.publish(msg)

        msg = String()
        msg.data = POI['NO_GRAPH_NODE']
        self.poi_pub.publish(msg)
