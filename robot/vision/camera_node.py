from __future__ import division, print_function

import cv2
from std_msgs.msg import Float32, String

from robot.common import POI

from .camera_base import Camera
from .mask import mask_image


class NodeCamera(Camera):
    """Camera class for detecting nodes."""

    # The minimum area of a contour required to be considered the goal.
    MIN_NODE_AREA = 20000
    MIN_POI_AREA = 5000
    REGION_OF_INTEREST = (slice(460, 480, None), slice(0, None, None))

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
        # Convert 0-360 range to 0-179 range.
        hue = 300 / 360 * 179
        purple_mask = mask_image(hsv_image, (hue - 15, 40, 100),
                                 (hue + 15, 255, 255))

        if self.verbose:
            cv2.namedWindow('Node P Mask', cv2.WINDOW_NORMAL)
            cv2.imshow('Node P Mask', purple_mask)

            cv2.namedWindow('P Mask Slice', cv2.WINDOW_NORMAL)
            cv2.imshow('P Mask Slice', purple_mask[self.REGION_OF_INTEREST])

        _, contours, _ = cv2.findContours(purple_mask, 1,
                                          cv2.CHAIN_APPROX_SIMPLE)
        _, poi_contours, _ = cv2.findContours(
            purple_mask[self.REGION_OF_INTEREST], 1, cv2.CHAIN_APPROX_SIMPLE)

        error = Float32()
        error.data = 0.0
        poi = String()
        poi.data = POI['NO_GRAPH_NODE']

        # If we find any contours, find the biggest and call that the goal.
        if contours:
            max_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            # print('goal area:', area)

            M = cv2.moments(max_contour)
            # If the contour area is bigger than some threshold, try to find
            # its centroid, if possible.
            if area > self.MIN_NODE_AREA and M['m00'] != 0:
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

        if poi_contours:
            max_contour = max(poi_contours, key=cv2.contourArea)
            area = cv2.contourArea(max_contour)
            if area >= self.MIN_POI_AREA:
                print('POI area:', area)
                poi.data = POI['GRAPH_NODE']

        self.error_pub.publish(error)
        self.poi_pub.publish(poi)
