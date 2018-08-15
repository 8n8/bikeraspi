import pygame
import pygame.camera
from pygame.locals import *
import time

pygame.init()
pygame.camera.init()


@profile
def main():
    camList = pygame.camera.list_cameras()

    if len(camList) != 1:
        print "The list of cameras doesn't have exactly two elements."
        return
    print "got cam names"

    handles = [
        pygame.camera.Camera(camName, (640, 480)) for camName in camList]

    for h in handles:
        h.start()

    repetitions = 100

    a = time.time()

    for j in range(repetitions):
        print "top of loop" + str(j)

        for i, h in enumerate(handles):
            im = h.get_image()
            pygame.image.save(im, "testIm" + str(i) + ".jpg")

    print (time.time() - a) / repetitions


main()
