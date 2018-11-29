from .camera_base import Camera


class LaneCamera(Camera):
    """Camera class for lane following."""

    def process_image(self, hsv_image):
        """Implement lane detection and publishes the lane centroid."""
        pass
