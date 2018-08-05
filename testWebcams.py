import pygame
import pygame.camera
from pygame.locals import *
import time

pygame.init()
pygame.camera.init()


def main():
    camList = pygame.camera.list_cameras()

    time.sleep(0.1)

    if len(camList) != 3:
        print "The list of cameras doesn't have exactly three elements."
        return
    print "got cam names"

    # handles = [
    #     pygame.camera.Camera(camName, (640, 480)) for camName in camList]

    handles = [pygame.camera.Camera(camName, (640, 480)) for camName in camList]

    print "got handles"
    for handle in handles:
        print "starting handle"
        handle.start()

    h0 = handles[0]
    h1 = handles[1]
    h2 = handles[2]

    print "handles started"

    time.sleep(0.1)

    repetitions = 100

    a = time.time()

    for _ in range(repetitions):
        print "top of loop"

        im0 = h0.get_image()
        pygame.image.save(im0, "testIm0.jpg")

        print "done pic 1"
        time.sleep(0.3)

        im1 = h1.get_image()
        print "b"
        pygame.image.save(im1, "testIm1.jpg")

        print "done pic 2"

        im2 = h2.get_image()
        pygame.image.save(im2, "testIm2.jpg")

        print "done pic 3"

    print (time.time() - a) / repetitions


main()
