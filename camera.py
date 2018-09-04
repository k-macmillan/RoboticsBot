import cv2 as cv
import numpy as np
import linefollow as linef


class Camera():
    def __init__(self):
        self.vid_capture = cv.VideoCapture(1)
        self.width = int(self.vid_capture.get(3))   # width float
        self.height = int(self.vid_capture.get(4)) # height float
        self.lf = linef.LineFollow(self.width, self.height)
        self.frame = 0
        self.adj_frame = 0



    def adjustSaturationBrightness(self):
        hsvImg = cv.cvtColor(self.frame,cv.COLOR_BGR2HSV)

        #multiple by a factor to change the saturation
        hsvImg[...,1] = hsvImg[...,1]*1.3

        #multiple by a factor of less than 1 to reduce the brightness 
        hsvImg[...,2] = hsvImg[...,2]*0.8

        self.adj_frame = cv.cvtColor(hsvImg,cv.COLOR_HSV2BGR)


    def loop(self):
        while (True):
            ret, self.frame = self.vid_capture.read()
            # self.adjustSaturationBrightness()     # not needed atm
            self.adj_frame = cv.GaussianBlur(self.frame.copy(), (5,5), 5)
            self.lf.UpdateAll(self.adj_frame)
            cv.imshow('Untouched Frame',self.frame)
            cv.imshow('Black Detection',self.lf.hsv_mask.mask)
            cv.imshow('Yellow Detection', self.lf.stop_mask.mask)
            cv.imshow("Road Slice", self.lf.road_slice)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        self.vid_capture.release()
        cv.destroyAllWindows()



if __name__ == "__main__":
    cam = Camera()
    cam.loop()