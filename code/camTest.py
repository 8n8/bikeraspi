import cv2

cam = cv2.VideoCapture(2)
s, img = cam.read()
if s:
    cv2.namedWindow("cam0", cv2.CV_WINDOW_AUTOSIZE)
    cv2.imshow("cam0", img)
    cv2.waitKey(0)
    cv2.destroyWindow("cam0")
    cv2.imwrite("cam0.jpt", img)
