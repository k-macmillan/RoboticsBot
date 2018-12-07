from __future__ import division, print_function

import cv2
import numpy as np


def mask_image(image, low_color, high_color):
    """Mask the given image.

    Filter out pixels that do not fall within the given HSV color ranges. Will
    attempt to denoise the produced mask.

    :param image: The image to mask.
    :type image: A 2D numpy array of (H, S, V) pixels.
    :param low_color: The low HSV value.
    :type low_color: A (H, S, V) tuple of integers.
    :param high_color: The high HSV value.
    :type high_color: A (H, S, V) tuple of integers.
    """
    # Convert the given tuples to numpy arrays.
    low_color = np.array(low_color)
    high_color = np.array(high_color)
    # Produce the mask.
    mask = cv2.inRange(image, low_color, high_color)
    # Denoise the mask.
    mask = cv2.erode(
        mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8)),
        iterations=1)
    mask = cv2.dilate(
        mask,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
        iterations=1)

    return mask
