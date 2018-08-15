import pygame
import pygame.camera
from pygame.locals import *
import time
import sys

pygame.init()
pygame.camera.init()


def main():
    camList = pygame.camera.list_cameras()

    if len(camList) != 3:
        print "The list of cameras doesn't have exactly three elements."
        return
    print "got cam names"

    arg = sys.argv[1]

    handle = pygame.camera.Camera(camList[int(arg)], (640, 480))
    handle.start()

    print "handle started"

    repetitions = 1000

    a = time.time()

    for i in range(repetitions):
        print str(i) + " of 1000" 

        im = handle.get_image()
        pygame.image.save(im, "testIm0.jpg")

    print (time.time() - a) / repetitions

main()
