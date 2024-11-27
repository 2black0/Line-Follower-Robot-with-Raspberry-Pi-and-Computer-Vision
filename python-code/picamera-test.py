from picamera import PiCamera
import time
import cv2

#Capture image using picamera module
camera = PiCamera()
time.sleep(0.1)
camera.capture('test.jpg')

#Read image from MicroSD card
image = cv2.imread('test.jpg', -1)

#Display image in an OpenCV window
cv2.namedWindow('Image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Image', 640, 480)
cv2.imshow('Image', image)
cv2.waitKey(0)