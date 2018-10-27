import cv2 as cv
from robot import linefollow


class Camera(object):
    """Open webcam and look for black and yellow objects."""

    def __init__(self, camera_index):
        """Create a camera object.

        :param camera_index: The index of the webcam to use.
        """
        self.vid_capture = cv.VideoCapture(camera_index)
        self.width = int(self.vid_capture.get(3))   # width float
        self.height = int(self.vid_capture.get(4)) # height float
        self.lf = linefollow.LineFollow(self.width, self.height)
        self.frame = 0
        self.adj_frame = 0

    def adjustSaturationBrightness(self):
        """Adjust the saturation of the current frame."""
        hsvImg = cv.cvtColor(self.frame, cv.COLOR_BGR2HSV)

        #multiple by a factor to change the saturation
        hsvImg[..., 1] = hsvImg[..., 1]*1.3

        #multiple by a factor of less than 1 to reduce the brightness
        hsvImg[..., 2] = hsvImg[..., 2]*0.8

        self.adj_frame = cv.cvtColor(hsvImg, cv.COLOR_HSV2BGR)

    def loop(self):
        """Read frames from the webcam."""
        while True:
            _, self.frame = self.vid_capture.read()
            # self.adjustSaturationBrightness()     # not needed atm
            self.adj_frame = cv.GaussianBlur(self.frame.copy(), (5, 5), 5)
            self.lf.UpdateAll(self.adj_frame)
            cv.imshow('Untouched Frame', self.frame)
            cv.imshow('Black Detection', self.lf.hsv_mask.mask)
            cv.imshow('Yellow Detection', self.lf.stop_mask.mask)
            cv.imshow("Road Slice", self.lf.road_slice)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        self.vid_capture.release()
        cv.destroyAllWindows()
