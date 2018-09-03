import cv2 as cv
import numpy as np
import linefollow as linef
import yellowcard as ycard


class Camera():
    def __init__(self):
        self.vid_capture = cv.VideoCapture(1)
        self.lf = linef.LineFollow()
        self.yc = ycard.YellowCard()

    def loop(self):
        while (True):
            ret, frame = self.vid_capture.read()
            self.lf.FindOffset(frame)
            cv.imshow('frame',frame)
            cv.imshow('mask',self.lf.hsv_mask.mask)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        self.vid_capture.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    cam = Camera()
    cam.loop()