from abc import ABCMeta, abstractmethod


class Camera(object):
    """An abstract base class for camera processing."""

    __metaclass__ = ABCMeta

    def __init__(self, publisher, verbose=False):
        """Create a base Camera that publishes on the given publisher.

        :param publisher: The ROS publisher to publish messages with.
        :type publisher: rospy.Publisher
        :param verbose: Should this Camera perfect the art of console spam,
        defaults to False
        :type verbose: bool, optional
        """
        self.publisher = publisher
        self.verbose = verbose

    @abstractmethod
    def process_image(self, hsv_image):
        """Process an HSV image.

        This function returns nothing, but publishes messages on the publisher
        this Camera was instantiated with.

        :param hsv_image: The HSV image to process.
        :type hsv_image: An OpenCV HSV image.
        """
        pass
