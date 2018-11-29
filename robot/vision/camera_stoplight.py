from .camera_base import Camera


class StoplightCamera(Camera):
    """Camera class for stoplight detection."""

    def process_image(self, hsv_image):
        """Publish a notification of a stoplight is encountered."""
        pass
