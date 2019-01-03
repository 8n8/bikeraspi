#import pygame
#import pygame.camera
#from pygame.locals import *
import time
import cv2

#pygame.init()
#pygame.camera.init()


def main():
    # camList = pygame.camera.list_cameras()

    # if len(camList) != 1:
    #     print "The list of cameras doesn't have exactly two elements."
    #     return
    # print "got cam names"

    # handles = [
    #     pygame.camera.Camera(camName, (640, 480)) for camName in camList]

    # for h in handles:
    #     h.start()

    handle = cv2.VideoCapture(1)
    if not handle.isOpened():
        print "Could not connect to webcam."
        return

    repetitions = 100

    a = time.time()

    for j in range(repetitions):
        print "top of loop" + str(j)

        #for i, h in enumerate(handles):
            #im = h.get_image()
            #pygame.image.save(im, "testIm" + str(i) + ".jpg")

        ok, photo = handle.read()
        if not ok:
            print "Could not take photo."
            return

        blackAndWhite = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)
        cv2.imwrite("testphoto.jpg", blackAndWhite)


    print (time.time() - a) / repetitions


main()
