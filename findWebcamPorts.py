import cv2
import time


def connectToCam(camNum):
    handle = cv2.VideoCapture(camNum)
    if not handle.isOpened():
        return None, "Could not connect to webcam {}".format(camNum)
    time.sleep(0.1)
    return handle, None

for i in range(-100, 100):
    print "Trying port " + str(i)
    h, err = connectToCam(i)
    if err is not None:
        continue
    print "No error with port " + str(i)
