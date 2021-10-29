from threading import Thread
import cv2
import numpy as np
from numpy import mean
from PIL import Image

#from cscore import CameraServer, VideoSource
kernel = np.ones((5, 5), np.uint8)

def contouring(frame):

    image = frame
    # image = cv2.imread('C:/Users/miles/PythonStuff/target.jpg')
    # grayscales the image
    #img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    out = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    target_image = cv2.inRange(out, (55, 133, 156), (140, 255, 255))
    img_edge = cv2.Canny(image,500,700)
    cv2.imshow('canny Image', img_edge)
    # Creating kernel


    img_erosion = cv2.erode(image, kernel, iterations=1)
    img_dilation = cv2.dilate(image, kernel, iterations=1)
    
    cv2.imshow('Input', image)
    cv2.imshow('Erosion', img_erosion)
    cv2.imshow('Dilation', img_dilation)
 
    cv2.waitKey(0)
    # Using cv2.erode() method 
    contours, hierarchy = cv2.findContours(target_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    largest = ""

    if len(contours) > 0:
        largest = contours[0]
        for contour in contours:
            if cv2.contourArea(contour) > cv2.contourArea(largest):
                largest = contour

    rect = cv2.minAreaRect(largest)
    center, _, _ = rect
    center_x, center_y = center
    print(center)

    yaw = 0

    M = cv2.moments(largest)
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    cv2.circle(image, (cX, cY), 7, (0, 0, 255), -1)
                
    cv2.drawContours(image, largest, -1, (255,0,255), 2)

    cv2.imshow('image window', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


contouring(cv2.imread('C:/Users/miles/PythonStuff/target.jpg'))



