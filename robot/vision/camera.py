from __future__ import division, print_function

from Tkinter import HORIZONTAL, Canvas, Label, Scale, Tk

import cv2
import numpy as np
import rospy as ros
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage

from robot.nodes import Node


# TODO: Remove the TKinter GUI because it doesn't play nice with the NodeManager
class Camera(Node):
    """A ROS node to receive and process the compressed camera feed from the robot."""

    def __init__(self, compressed_topic='/geekbot/webcam/image_raw/compressed'):
        """Create a Camera ROS node."""
        super(Camera, self).__init__(name='Camera')
        ros.Subscriber(compressed_topic, CompressedImage, self.callback)

        self.bridge = CvBridge()

        # The upper and lower values for the HSV mask.
        self.l_hue = 0
        self.l_sat = 0
        self.l_val = 0
        self.h_hue = 0
        self.h_sat = 0
        self.h_val = 0

        self.tk_init()

        # The minimum area for a contour
        self.min_area = 50 * 50

    def stop(self):
        """Signal handler to stop the ROS node."""
        cv2.destroyAllWindows()

    def run(self):
        """Run the ROS node in a separate process."""
        # Initialize and start the TKinter window for updating the HSV mask.
        self.tk_init()
        super(Camera, self).run()

    def callback(self, msg):
        """Receive a compressed frame from the camera.

        :param msg: The compressed frame.
        :type msg: sensor_msgs.msg.CompressedImage
        """
        try:
            # Decompress the message into an openCV frame.
            bgr_frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            print(e)

        # Convert BGR to HSV
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)

        # Get the HSV bounds from the TKinter window.
        low = np.array([self.l_hue, self.l_sat, self.l_val])
        high = np.array([self.h_hue, self.h_sat, self.h_val])

        # Mask the HSV frame.
        mask = cv2.inRange(hsv_frame, low, high)
        # Filter out noise.
        mask = self.denoise_mask(mask)
        # Use the mask to filter the BGR frame.
        filtered = cv2.bitwise_and(bgr_frame, bgr_frame, mask=mask)

        # Find the biggest contour in the mask larger than the minimum.
        # Draw the contour and its centroid.
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest = self.largest_contour(contours)
            if largest is not None:
                cv2.drawContours(filtered, largest, -1, (0, 255, 0), 3)
                # TODO: Do something with the centroid?
                centroid = Point()
                centroid.x, centroid.y = self.centroid(largest)

                cv2.circle(filtered, (centroid.x, centroid.y), 10, (0, 0, 255))

        cv2.namedWindow('Filtered', cv2.WINDOW_NORMAL)
        cv2.imshow('Filtered', filtered)
        cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
        cv2.imshow('Original', bgr_frame)

        cv2.waitKey(10)

    @staticmethod
    def denoise_mask(mask):
        """Denoise a mask by eroding small objects and inflating them.

        :param mask: The HSV image mask to denoise.
        :type mask: A boolean mask of everything between HSV bounds.
        """
        temp = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)), iterations=1)
        # TODO: Pass mask or temp?!
        temp = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
        return temp

    def largest_contour(self, contours):
        """Find the largest contour from a list."""
        largest = contours[0]
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(largest):
                largest = contour
        if cv2.contourArea(largest) < self.min_area:
            return None
        return largest

    @staticmethod
    def centroid(contour):
        """Find the centroid of the given contour.

        :param contour: The contour the find the centroid of.
        :type contour: An openCV contour.
        """
        M = cv2.moments(contour)
        return int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])

    def tk_init(self):
        """Initialize the TKinter window.

        This window allows setting the HSV mask using sliders.
        """
        self.root = Tk()
        self.window = Canvas(self.root, width=640, height=480+160, bd=10, bg='white')

        Label(self.root, text="Low  Hue:").grid(column=0, row=1)
        self.low_hue = Scale(self.root, from_=0, to=180, length=480, orient=HORIZONTAL)
        self.low_hue.grid(column=1, row=1)

        Label(self.root, text="Low  Sat:").grid(column=0, row=2)
        self.low_sat = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
        self.low_sat.grid(column=1, row=2)

        Label(self.root, text="Low  Val:").grid(column=0, row=3)
        self.low_val = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
        self.low_val.grid(column=1, row=3)

        Label(self.root, text="High Hue:").grid(column=0, row=4)
        self.high_hue = Scale(self.root, from_=0, to=180, length=480, orient=HORIZONTAL)
        self.high_hue.grid(column=1, row=4)

        Label(self.root, text="High Sat:").grid(column=0, row=5)
        self.high_sat = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
        self.high_sat.grid(column=1, row=5)

        Label(self.root, text="High Val:").grid(column=0, row=6)
        self.high_val = Scale(self.root, from_=0, to=255, length=480, orient=HORIZONTAL)
        self.high_val.grid(column=1, row=6)

        # Initialize these values in case the callback gets pulled before Tk updates
        self.l_hue = self.low_hue.get()
        self.l_sat = self.low_sat.get()
        self.l_val = self.low_val.get()
        self.h_hue = self.high_hue.get()
        self.h_sat = self.high_sat.get()
        self.h_val = self.high_val.get()

    def tk_update(self):
        """Update the TKinter window."""
        # Run the GUI event loop briefly.
        self.root.update()

        # Update the HSV mask values from the sliders.
        self.l_hue = self.low_hue.get()
        self.l_sat = self.low_sat.get()
        self.l_val = self.low_val.get()
        self.h_hue = self.high_hue.get()
        self.h_sat = self.high_sat.get()
        self.h_val = self.high_val.get()
