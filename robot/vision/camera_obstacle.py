from __future__ import division, print_function

import cv2
import numpy as np

from .camera_base import Camera


class ObstacleCamera(Camera):
    """Camera class for obstacle detection."""

    # How much do we blur the image
    BLUR_KERNEL = (5, 5)
    # Arbitrary cutoff for determining if an object is detected
    YELLOW_CUTOFF = 500

    def process_image(self, hsv_image):
        """Publish a notification if an obstacle is encountered."""
        sensitivity = 5
        y_low = np.array([25 - sensitivity, 50, 50])
        y_high = np.array([25 + sensitivity, 255, 255])

        self.mask = cv2.inRange(blurred, y_low, y_high)
        print('processing...')
        centroid, closest = self.__identifyObstacle()
        print('Centroid: ({}, {})'.format(centroid[0], centroid[1]))
        print('Closest : ({}, {})'.format(closest[0], closest[1]))

        # Publish obstacle
        # May require Float32MultiArray for the brain to process this one...

        if self.verbose:
            # Mask out only the reds. Everything else will be black.
            masked = cv2.bitwise_and(hsv_image, hsv_image, mask=self.mask)
            cv2.imshow('Obstacles', cv2.cvtColor(masked, cv2.COLOR_HSV2BGR))

    def __identifyObstacle(self):
        """Identify an obstacle if it is in the frame.

        The assumption is that the closest pixel in this matrix is the
        obstacle and all obstacles are one cluster. This method is hefty
        because the mask contains pixel data, not point data so we have to
        analyze the pixel data to determine if it should be added to our
        centroid calculation.
        """
        # Find centroid of object and closest pixel
        camera = (320, 479)
        pt_count = 0
        x_sum = 0.0
        y_sum = 0.0
        closest = (-1, -1)
        best_dist = float('inf')

        for x in range(480):
            for y in range(640):
                # We are only concerned with threshold pixels
                if self.mask[x][y] == 255:
                    # Add point to x/y sums
                    x_sum += x
                    y_sum += y
                    pt_count += 1

                    # Distance^2 from point to camera
                    dist = self.__distanceSqrd(camera, (x, y))
                    if dist < best_dist:
                        best_dist = dist
                        closest = (x, y)
        if pt_count != 0:
            centroid = (x_sum / pt_count, y_sum / pt_count)
        else:
            centroid = closest
        return centroid, closest

    def __distanceSqrd(self, a, b):
        return (a[0] - b[0])**2 + (a[1] - b[1])**2
