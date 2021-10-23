#!/usr/bin/env python3

'''
[{"name":"connect_verbose","value":1},{"name":"contrast","value":12},{"name":"saturation","value":89},{"name":"hue","value":35},{"name":"white_balance_automatic","value":false},{"name":"exposure","value":1},{"name":"gain_automatic","value":false},{"name":"gain","value":31},{"name":"horizontal_flip","value":false},{"name":"vertical_flip","value":false},{"name":"power_line_frequency","value":0},{"name":"sharpness","value":0},{"name":"auto_exposure","value":1}]

joes setings
[{"name":"connect_verbose","value":1},{"name":"contrast","value":44},{"name":"saturation","value":64},{"name":"hue","value":18},{"name":"white_balance_automatic","value":false},{"name":"exposure","value":1},{"name":"gain_automatic","value":false},{"name":"gain","value":31},{"name":"horizontal_flip","value":false},{"name":"vertical_flip","value":false},{"name":"power_line_frequency","value":0},{"name":"sharpness","value":0},{"name":"auto_exposure","value":1}]
'''

import json, time, sys, os 
from threading import Thread
import socket, queue, threading, cv2, math 
import numpy as np
from numpy import mean

# Image Camera Size (Pixels)
Camera_Image_Width = 640
Camera_Image_Height = 480

centerX = (Camera_Image_Width / 2) - .5
centerY = (Camera_Image_Height/2) - .5

# Aspect Ratio
HorizontalAspect = 4
VerticalAspect = 3
DiagonalAspect = math.hypot(HorizontalAspect, VerticalAspect)


# Ball HSV Values
Ball_HSV_Lower = np.array([13, 67, 188])
Ball_HSV_Upper = np.array([62, 255, 255])

#Non changing distance variables
#PSEYE WIDE ANGLE FOV = 75, CLOSE ANGLE = 56
FOV = 75

# High goal height = 8 feet, 2.25 inches, actual height of center goal is 96.25,
# Centroid of the tape is ~87.75 inches (center of height of tape)
# tape is 1 ft 5inches, 17 inches/2 = 8.5 inches. 96.25-8.5 gives 87.75

#Camera height is 37.5 inches
targetHeightInches = 50.25

camPixelWidth = 640
# target reflective tape width in feet (3 feet, 3 & 1/4 inch) ~3.27
Tft = 3.27

# theta = 1/2 FOV,
tanFOV = math.tan(FOV / 2)



#Constraint values
# ratio values - detects feeder station. Doing and not when doing ratio checks will ignore them
rat_low = 1.5
rat_high = 5

#Solitity compares the hull vs contour, and looks at the difference in filled area
#Works on a system of %
solidity_low = .1
solidity_high = .3

#Vertices is acts as "length"
minArea = 10
minWidth = 20
maxWidth = 250
minHeight = 20
maxHeight = 500
maxVertices = 200
minVertices = 0

hsv_threshold_hue = [15, 166]
hsv_threshold_saturation = [71, 255]
hsv_threshold_value = [39, 255]



# List will go in order [x of target position, y of target position, yaw, distance, ]
sendValues = np.array([None] * 4)
distanceHoldValues = np.array([])
shootingDistance = 0
outlierCount = 0
run_count = 0
starttime = 0

# import the necessary packages
import datetime

# Queue of Packets
# Thread Safe.. Packets being sent to robot are placed here!
PacketQueue = queue.Queue()






__hsv_threshold_0_hue = [30.755395683453237, 80.35681042506978]
__hsv_threshold_0_saturation = [87.14028776978417, 255.0]
__hsv_threshold_0_value = [57.32913669064748, 255.0]

hsv_threshold_0_output = None


__hsv_threshold_1_hue = [0.0, 27.95221843003414]
__hsv_threshold_1_saturation = [52.74280575539568, 255.0]
__hsv_threshold_1_value = [71.08812949640287, 255.0]

hsv_threshold_1_output = None

__cv_bitwise_not_src1 = hsv_threshold_1_output

cv_bitwise_not_output = None

__mask_input = hsv_threshold_0_output
__mask_mask = cv_bitwise_not_output

mask_output = None

__find_contours_input = mask_output
__find_contours_external_only = True

find_contours_output = None

__filter_contours_contours = find_contours_output
__filter_contours_min_area = 10.0
__filter_contours_min_perimeter = 0.0
__filter_contours_min_width = 20.0
__filter_contours_max_width = 1000.0
__filter_contours_min_height = 20.0
__filter_contours_max_height = 60.0
__filter_contours_solidity = [10.166358595194085, 30.313497671010627]
__filter_contours_max_vertices = 100.0
__filter_contours_min_vertices = 30.0
__filter_contours_min_ratio = 1.5
__filter_contours_max_ratio = 5.0

filter_contours_output = None

__convex_hulls_contours = filter_contours_output

convex_hulls_output = None




def __hsv_threshold(input, hue, sat, val):
    """Segment an image based on hue, saturation, and value ranges.
    Args:
        input: A BGR numpy.ndarray.
        hue: A list of two numbers the are the min and max hue.
        sat: A list of two numbers the are the min and max saturation.
        lum: A list of two numbers the are the min and max value.
    Returns:
        A black and white numpy.ndarray.
    """
    out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
    return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))


def __cv_bitwise_not(src1):
    """Computes the per element inverse of an image.
    Args:
        src1: A numpy.ndarray.
    Returns:
        The inverse of the numpy.ndarray.
    """
    return cv2.bitwise_not(src1)


def __mask(input, mask):
    """Filter out an area of an image using a binary mask.
    Args:
        input: A three channel numpy.ndarray.
        mask: A black and white numpy.ndarray.
    Returns:
        A three channel numpy.ndarray.
    """
    return cv2.bitwise_and(input, input, mask=mask)


def __find_contours(input, external_only):
    """Sets the values of pixels in a binary image to their distance to the nearest black pixel.
    Args:
        input: A numpy.ndarray.
        external_only: A boolean. If true only external contours are found.
    Return:
        A list of numpy.ndarray where each one represents a contour.
    """
    if(external_only):
        mode = cv2.RETR_EXTERNAL
    else:
        mode = cv2.RETR_LIST
    method = cv2.CHAIN_APPROX_SIMPLE
    im2, contours, hierarchy =cv2.findContours(input, mode=mode, method=method)
    return contours

def __filter_contours(input_contours, min_area, min_perimeter, min_width, max_width,
                    min_height, max_height, solidity, max_vertex_count, min_vertex_count,
                    min_ratio, max_ratio):
    """Filters out contours that do not meet certain criteria.
    Args:
        input_contours: Contours as a list of numpy.ndarray.
        min_area: The minimum area of a contour that will be kept.
        min_perimeter: The minimum perimeter of a contour that will be kept.
        min_width: Minimum width of a contour.
        max_width: MaxWidth maximum width.
        min_height: Minimum height.
        max_height: Maximimum height.
        solidity: The minimum and maximum solidity of a contour.
        min_vertex_count: Minimum vertex Count of the contours.
        max_vertex_count: Maximum vertex Count.
        min_ratio: Minimum ratio of width to height.
        max_ratio: Maximum ratio of width to height.
    Returns:
        Contours as a list of numpy.ndarray.
    """
    output = []
    for contour in input_contours:
        x,y,w,h = cv2.boundingRect(contour)
        if (w < min_width or w > max_width):
            continue
        if (h < min_height or h > max_height):
            continue
        area = cv2.contourArea(contour)
        if (area < min_area):
            continue
        if (cv2.arcLength(contour, True) < min_perimeter):
            continue
        hull = cv2.convexHull(contour)
        solid = 100 * area / cv2.contourArea(hull)
        if (solid < solidity[0] or solid > solidity[1]):
            continue
        if (len(contour) < min_vertex_count or len(contour) > max_vertex_count):
            continue
        ratio = (float)(w) / h
        if (ratio < min_ratio or ratio > max_ratio):
            continue
        output.append(contour)
    return output

def __convex_hulls(input_contours):
    """Computes the convex hulls of contours.
    Args:
        input_contours: A list of numpy.ndarray that each represent a contour.
    Returns:
        A list of numpy.ndarray that each represent a contour.
    """
    output = []
    for contour in input_contours:
        output.append(cv2.convexHull(contour))
    return output










# Creates a socket
class SocketWorker(threading.Thread):
    def __init__(self, q, *args, **kwargs):
        self.queue = q
        super().__init__(*args, **kwargs)

        # Initialize Socket Connect
        SocketHost = "10.1.38.2"
        SocketPort = 5800
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((SocketHost, SocketPort))

    def run(self):
        while True:
            try:
                packet = self.queue.get()
                # Convert Packet to JSON
                # Send Message Over Socket
                try:
                    data_string = json.dumps(packet)
                    self.sock.sendall(data_string.encode())
                except Exception as e:
                    print("Socket Exception " + str(e))
            except Exception as e1:
                pass


# Class to examine Frames per second of camera stream. Currently not used.
class FPS:
    def __init__(self):
        # store the start time, end time, and total number of frames
        # that were examined between the start and end intervals
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        # start the timer
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        # stop the timer
        self._end = datetime.datetime.now()

    def update(self):
        # increment the total number of frames examined during the
        # start and end intervals
        self._numFrames += 1

    def elapsed(self):
        # return the total number of seconds between the start and
        # end interval
        return (self._end - self._start).total_seconds()

    def fps(self):
        # compute the (approximate) frames per second
        return self._numFrames / self.elapsed()


# class that runs separate thread for showing video,
class VideoShow:
    """
    Class that continuously shows a frame using a dedicated thread.
    """

    def __init__(self, imgWidth, imgHeight, cameraServer, frame=None, name='stream'):
        self.outputStream = cameraServer.putVideo(name, imgWidth, imgHeight)
        self.frame = frame
        self.stopped = False

    def start(self):
        Thread(target=self.show, args=()).start()
        return self

    def show(self):
        while not self.stopped:
            self.outputStream.putFrame(self.frame)

    def stop(self):
        self.stopped = True

    def notifyError(self, error):
        self.outputStream.notifyError(error)


class WebcamVideoStream:
    def __init__(self, camera, cameraServer, frameWidth, frameHeight, name="WebcamVideoStream"):
        # initialize the video camera stream and read the first frame
        # from the stream

        # Automatically sets exposure to 0 to track tape
        self.webcam = camera
        # print("SETTING EXPOSURE ")
        # print(self.webcame.exposure)

        # self.webcam.setExposureManual(0)
        # Some booleans so that we don't keep setting exposure over and over to the same value
        # self.autoExpose = False
        # self.prevValue = self.autoExpose
        # Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        # Gets the video
        self.stream = cameraServer.getVideo(camera=camera)
        (self.timestamp, self.img) = self.stream.grabFrame(self.img)

        # initialize the thread name
        self.name = name

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        t = Thread(target=self.update, name=self.name, args=())
        t.daemon = True
        t.start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            # Boolean logic we don't keep setting exposure over and over to the same value
            '''
            if self.autoExpose:
                self.webcam.setExposureAuto()
            else:
                self.webcam.setExposureManual(0)
            '''
            # gets the image and timestamp from cameraserver
            (self.timestamp, self.img) = self.stream.grabFrame(self.img)

    def read(self):
        # return the frame most recently read
        return self.timestamp, self.img

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def getError(self):
        return self.stream.getError()


###################### PROCESSING OPENCV ################################

# Angles in radians

# image size ratioed to 16:9
image_width = 640
image_height = 480

# Playstation Eye
# Datasheet: https://en.wikipedia.org/wiki/PlayStation_Eye
diagonalView = math.radians(75)

# 16:9 aspect ratio
horizontalAspect = 4
verticalAspect = 3

# Reasons for using diagonal aspect is to calculate horizontal field of view.
diagonalAspect = math.hypot(horizontalAspect, verticalAspect)
# Calculations: http://vrguy.blogspot.com/2013/04/converting-diagonal-field-of-view-and.html
horizontalView = math.atan(math.tan(diagonalView / 2) * (horizontalAspect / diagonalAspect)) * 2
verticalView = math.atan(math.tan(diagonalView / 2) * (verticalAspect / diagonalAspect)) * 2

# Focal Length calculations: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_165
H_FOCAL_LENGTH = image_width / (2 * math.tan((horizontalView / 2)))
V_FOCAL_LENGTH = image_height / (2 * math.tan((verticalView / 2)))
# blurs have to be odd
green_blur = 0
orange_blur = 27

# define range of green of retroreflective tape in HSV


# lower_green = np.array([0,220,25])
# upper_green = np.array([101, 255, 255])

lower_green = np.array([65, 36.131, 83])
upper_green = np.array([98, 255, 195])

# define range of orange from cargo ball in HSV
lower_orange = np.array([0, 193, 92])
upper_orange = np.array([23, 255, 255])


# Flip image if camera mounted upside down
def flipImage(frame):
    return cv2.flip(frame, -1)

# Masks the video based on a range of hsv colors
# Takes in a frame, range of color, and a blurred frame, returns a masked frame
def threshold_video(lower_color, upper_color, blur):
    global hsv_threshold_hue
    global hsv_threshold_saturation
    global hsv_threshold_value
    out = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(out, (hsv_threshold_hue[0], hsv_threshold_saturation[0], hsv_threshold_value[0]),
                       (hsv_threshold_hue[1], hsv_threshold_saturation[1], hsv_threshold_value[1]))

    # Returns the masked imageBlurs video to smooth out image

    return mask


vals_to_send = np.array([None] * 4)


# Finds the tape targets from the masked image and displays them on original stream + network tales
def findTargets(frame, mask, value_array, centerX, centerY):
    # Finds contours
    _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_TC89_KCOS)
    # Take each frame
    # Gets the shape of video
    # Gets center of height and width
    # Copies frame and stores it in image

    # Processes the contours, takes in (contours, output_image, (centerOfImage)
    if len(contours) != 0:
        value_array = findTape(contours, frame, centerX, centerY)
    else:
        # No Contours!
        pass

    # Shows the contours overlayed on the original video
    return value_array


# Draws Contours and finds center and yaw of orange ball
# centerX is center x coordinate of image
# centerY is center y coordinate of image
def findBall(contours, image, centerX, centerY):
    screenHeight, screenWidth, channels = image.shape;
    # Seen vision targets (correct angle, adjacent to each other)
    cargo = []

    if len(contours) > 0:
        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

        biggestCargo = []
        for cnt in cntsSorted:
            x, y, w, h = cv2.boundingRect(cnt)
            aspect_ratio = float(w) / h
            # Get moments of contour; mainly for centroid
            M = cv2.moments(cnt)
            # Get convex hull (bounding polygon on contour)
            hull = cv2.convexHull(cnt)
            # Calculate Contour area
            cntArea = cv2.contourArea(cnt)
            # Filters contours based off of size
            if (checkBall(cntArea, aspect_ratio)):
                ### MOSTLY DRAWING CODE, BUT CALCULATES IMPORTANT INFO ###
                # Gets the centeroids of contour
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                else:
                    cx, cy = 0, 0
                if (len(biggestCargo) < 3):

                    ##### DRAWS CONTOUR######
                    # Gets rotated bounding rectangle of contour
                    rect = cv2.minAreaRect(cnt)
                    # Creates box around that rectangle
                    box = cv2.boxPoints(rect)
                    # Not exactly sure
                    box = np.int0(box)
                    # Draws rotated rectangle
                    cv2.drawContours(image, [box], 0, (23, 184, 80), 3)

                    # Draws a vertical white line passing through center of contour
                    cv2.line(image, (cx, screenHeight), (cx, 0), (255, 255, 255))
                    # Draws a white circle at center of contour
                    cv2.circle(image, (cx, cy), 6, (255, 255, 255))

                    # Draws the contours
                    cv2.drawContours(image, [cnt], 0, (23, 184, 80), 1)

                    # Gets the (x, y) and radius of the enclosing circle of contour
                    (x, y), radius = cv2.minEnclosingCircle(cnt)
                    # Rounds center of enclosing circle
                    center = (int(x), int(y))
                    # Rounds radius of enclosning circle
                    radius = int(radius)
                    # Makes bounding rectangle of contour
                    rx, ry, rw, rh = cv2.boundingRect(cnt)

                    # Draws countour of bounding rectangle and enclosing circle in green
                    cv2.rectangle(image, (rx, ry), (rx + rw, ry + rh), (23, 184, 80), 1)

                    cv2.circle(image, center, radius, (23, 184, 80), 1)

                    # Appends important info to array
                    if not biggestCargo:
                        biggestCargo.append([cx, cy])
                    elif [cx, cy, cnt] not in biggestCargo:
                        biggestCargo.append([cx, cy])

        # Check if there are cargo seen
        if (len(biggestCargo) > 0):
            # Sorts targets based on x coords to break any angle tie
            biggestCargo.sort(key=lambda x: math.fabs(x[0]))
            closestCargo = min(biggestCargo, key=lambda x: (math.fabs(x[0] - centerX)))
            xCoord = closestCargo[0]
            finalTarget = calculateYaw(xCoord, centerX, H_FOCAL_LENGTH)
            print("Yaw: " + str(finalTarget))
            # Puts the yaw on screen
            # Draws yaw of target + line where center of target is
            cv2.putText(image, "Yaw: " + str(finalTarget), (40, 40), cv2.FONT_HERSHEY_COMPLEX, .6,
                        (255, 255, 255))
            cv2.line(image, (int(xCoord), screenHeight), (int(xCoord), 0), (255, 0, 0), 2)

            currentAngleError = finalTarget

        cv2.line(image, (int(centerX), screenHeight), (int(centerX), 0), (255, 255, 255), 2)

        return image


# Draws Contours and finds center and yaw of vision targets
# centerX is center x coordinate of image
# centerY is center y coordinate of image


def findTape(contours, image, centerX, centerY):
    sendValues = [None] * 4
    screenHeight, screenWidth, channels = image.shape;
    # Seen vision targets (correct angle, adjacent to each other)
    targets = []
    if len(contours) >= 2:
        # Sort contours by area size (biggest to smallest)
        cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)
        biggestCnts = []
        for cnt in cntsSorted:
            # Get moments of contour; mainly for centroid
            M = cv2.moments(cnt)
            # Get convex hull (bounding polygon on contour)
            hull = cv2.convexHull(cnt)
            # Calculate Contour area
            cntArea = cv2.contourArea(cnt)
            # calculate area of convex hull
            hullArea = cv2.contourArea(hull)

            perimeter = cv2.arcLength(cnt, True)
            approxCurve = cv2.approxPolyDP(cnt, perimeter * .01, True)


            if cntArea != 0 and hullArea != 0:
                mySolidity = float (cntArea)/hullArea
            else:
                mySolidity = 1000

            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w) / h
            # Filters contours based off of size
            if len(approxCurve) >= 8 and len(approxCurve) < 10 and (cntArea > minArea) and (mySolidity > solidity_low) and (mySolidity < solidity_high) and (w > minWidth) and (w < maxWidth) and (h > minHeight) and (h < maxHeight) and (checkContours(cntArea, hullArea, ratio, cnt)):
                # Next three lines are for debugging the contouring
                contimage = cv2.drawContours(image, cnt, -1, (0, 255, 0), 3)

                #cv2.imwrite("1drawncontours.jpg", contimage)
                #time.sleep(1)
                #print("writing image")

                ### MOSTLY DRAWING CODE, BUT CALCULATES IMPORTANT INFO ###
                # Gets the centeroids of contour
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    distCY = 540 - cy
                    myDistFeet = (calculateDistanceFeet(w))

                    ###### New code that has an averaged shooting distance to avoid outliers

                    #global run_count
                    global distanceHoldValues
                    global shootingDistance
                    global outlierCount
                    global run_count

                    #fills list to avoid errors
                    '''
                    if outlierCount >= 5:
                        distanceHoldValues = []

                    if len(distanceHoldValues) < 5:
                        distanceHoldValues = [myDistFeet] * 5

                    if abs(myDistFeet-mean(distanceHoldValues)) > 1.5:
                        outlierCount = outlierCount + 1
                        myDistFeet = None

                    else:
                        outlierCount = 0
                        distanceHoldValues.pop()
                        distanceHoldValues.append(myDistFeet)
                        myDistFeet = abs(myDistFeet)
                    '''

                    '''
                    run_count = run_count + 1
                    if run_count % 100 == 0:
                        starttime = time.time()
                        print(starttime)
                    '''
                    #print(myDistFeet)

                    sendValues[0] = cx
                    sendValues[1] = cy
                    sendValues[3] = myDistFeet
                    #print(sendValues[3])

                else:
                    cx, cy = 0, 0
                if (len(biggestCnts) < 13):
                    #### CALCULATES ROTATION OF CONTOUR BY FITTING ELLIPSE ##########
                    rotation = getEllipseRotation(image, cnt)

                    # Appends important info to array
                    if not biggestCnts:
                        biggestCnts.append([cx, cy, rotation])
                    elif [cx, cy, rotation] not in biggestCnts:
                        biggestCnts.append([cx, cy, rotation])

        # Sorts array based on coordinates (leftmost to rightmost) to make sure contours are adjacent
        biggestCnts = sorted(biggestCnts, key=lambda x: x[0])
        # Target Checking
        for i in range(len(biggestCnts) - 1):
            # Rotation of two adjacent contours
            tilt1 = biggestCnts[i][2]
            tilt2 = biggestCnts[i + 1][2]

            # x coords of contours
            cx1 = biggestCnts[i][0]
            cx2 = biggestCnts[i + 1][0]

            cy1 = biggestCnts[i][1]
            cy2 = biggestCnts[i + 1][1]
            # If contour angles are opposite
            if (np.sign(tilt1) != np.sign(tilt2)):
                centerOfTarget = math.floor((cx1 + cx2) / 2)
                # ellipse negative tilt means rotated to right
                # Note: if using rotated rect (min area rectangle)
                #      negative tilt means rotated to left
                # If left contour rotation is tilted to the left then skip iteration
                if (tilt1 > 0):
                    if (cx1 < cx2):
                        continue
                # If left contour rotation is tilted to the left then skip iteration
                if (tilt2 > 0):
                    if (cx2 < cx1):
                        continue
                # Angle from center of camera to target (what you should pass into gyro)
                yawToTarget = calculateYaw(centerOfTarget, centerX, H_FOCAL_LENGTH)
                # Make sure no duplicates, then append
                if not targets:
                    targets.append([centerOfTarget, yawToTarget])
                elif [centerOfTarget, yawToTarget] not in targets:
                    targets.append([centerOfTarget, yawToTarget])
    # Check if there are targets seen
    if len(targets) > 0:
        # Sorts targets based on x coords to break any angle tie
        targets.sort(key=lambda x: math.fabs(x[0]))
        finalTarget = min(targets, key=lambda x: math.fabs(x[1]))
        print("finaltarget is:", finalTarget)
        # Puts the yaw on screen
        # Draws yaw of target + line where center of target is
        cv2.putText(image, "Yaw: " + str(finalTarget[1]), (40, 40), cv2.FONT_HERSHEY_COMPLEX, .6,
                    (255, 255, 255))

        currentAngleError = finalTarget[1]

    # print("TapeYaw: " + str(currentAngleError))

    cv2.line(image, (round(centerX), screenHeight), (round(centerX), 0), (255, 255, 255), 2)

    # cv2.imwrite("latest.jpg", image);

    return sendValues


# Checks if tape contours are worthy based off of contour area and (not currently) hull area
def checkContours(cntSize, hullSize, aspRatio, contour):
    return cntSize > (image_width / 6) and (len(contour) > minVertices) and (len(contour) < maxVertices) and not (aspRatio < rat_low or aspRatio > rat_high)

# Checks if ball contours are worthy based off of contour area and (not currently) hull area
def checkBall(cntSize, cntAspectRatio):
    return (cntSize > (image_width / 2)) and (round(cntAspectRatio) == 1)


# Forgot how exactly it works, but it works!
def translateRotation(rotation, width, height):
    if (width < height):
        rotation = -1 * (rotation - 90)
    if (rotation > 90):
        rotation = -1 * (rotation - 180)
    rotation *= -1
    return round(rotation)


def calculateDistanceFeet(targetPixelWidth):
    # d = Tft*FOVpixel/(2*Tpixel*tanΘ)
    #Target width in feet *
    distEst = Tft * camPixelWidth / (2 * targetPixelWidth * tanFOV)

    # Unsure as to what measurement distEst is producing in the above line, but multiplying it by .32 will return your distance in feet
    distEstFeet = distEst * .32
    #distEstInches = distEstFeet *.32*12
    return (abs(distEstFeet))


# Uses trig and focal length of camera to find yaw.
# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculateYaw(pixelX, centerX, hFocalLength):
    yaw = math.degrees(math.atan((pixelX - centerX) / hFocalLength))
    return round(yaw)


# Grabs Countours Based on Version
def grab_contours(cnts):
    # if the length the contours tuple returned by cv2.findContours
    # is '2' then we are using either OpenCV v2.4, v4-beta, or
    # v4-official
    if len(cnts) == 2:
        cnts = cnts[0]

    # if the length of the contours tuple is '3' then we are using
    # either OpenCV v3, v4-pre, or v4-alpha
    elif len(cnts) == 3:
        cnts = cnts[1]

    # otherwise OpenCV has changed their cv2.findContours return
    # signature yet again and I have no idea WTH is going on
    else:
        raise Exception(("Contours tuple must have length 2 or 3, "
                         "otherwise OpenCV changed their cv2.findContours return "
                         "signature yet again. Refer to OpenCV's documentation "
                         "in that case"))

    # return the actual contours array
    return cnts


# Perform a Mask on the ball
# use a range of colors around the ball color to account for lighting
def MaskBall(frame):
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, Ball_HSV_Lower, Ball_HSV_Upper)
    # remove any small blobs left in the mask
    # mask = cv2.erode(mask, None, iterations=2)
    # mask = cv2.dilate(mask, None, iterations=2)
    return mask


def CheckBall(CntSize, CntAspectRatio):
    return (CntSize > (Camera_Image_Width / 2)) and (round(CntAspectRatio) == 1)


# For Finding Power Cells
def findBalls(frame):
    FoundBalls = []  # Store and Return Tracked Balls
    ContourList = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ContourList = grab_contours(ContourList)

    # Proceed if we have contours
    if len(ContourList) > 0:
        # Sort Contours by Area Size (Largest -> Smallest)
        SortedContours = sorted(ContourList, key=lambda x: cv2.contourArea(x), reverse=True)

        # Goal is to return the largest ball
        FoundBall = None

        # Loop through Contour, Large to Smallest in Area
        for c in SortedContours:
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            x_br, y_br, w_br, h_br = cv2.boundingRect(c)
            # Image Moment is a particular weighted average of image pixel intensities
            # We can use it to find the center
            M = cv2.moments(c)
            center = 0
            centerX = 0
            try:
                cy = int(M["m01"] / M["m00"])
                cx = int(M["m10"] / M["m00"])
                center = (cx, cy)
            except:
                continue

            CntArea = cv2.contourArea(c)

            # Ignore small contours
            if CntArea < 150:
                continue

            cnt_aspect_ratio = float(w_br) / h_br
            AspectRatioCheck = (round(cnt_aspect_ratio) == 1)

            OkayBall = AspectRatioCheck
            if OkayBall == True:
                # This is a Target!
                #               Lets calculate now!
                ball = {}
                ball['Type'] = 1
                ball['CX'] = cx
                ball['CY'] = cy

                finalTarget = calculateYaw(cx, centerX, H_FOCAL_LENGTH)
                ball['Yaw'] = finalTarget
                ball['Size'] = CntArea

                # Set our Found Ball
                if FoundBall == None:
                    FoundBall = ball
                elif FoundBall['Size'] < ball['Size']:
                    FoundBall = ball
                print("Tracking Ball: " + str(ball['Yaw']))

        # Check if we have a found ball for this frame
        if not FoundBall == None:
            # print("Found Ball!")
            # print(FoundBall)(
            PacketQueue.put_nowait(FoundBall)
    return FoundBalls


# Proccesses each frame of the image
# boolean to whether we want ball tracking or tape tracking
def ProcessFrame(frame, tape):
    if (tape == True):
        source0 = frame

        # Step HSV_Threshold0:
        __hsv_threshold_0_input = source0
        (hsv_threshold_0_output) = __hsv_threshold(__hsv_threshold_0_input, __hsv_threshold_0_hue, __hsv_threshold_0_saturation, __hsv_threshold_0_value)

        # Step HSV_Threshold1:
        __hsv_threshold_1_input = source0
        (hsv_threshold_1_output) = __hsv_threshold(__hsv_threshold_1_input, __hsv_threshold_1_hue, __hsv_threshold_1_saturation, __hsv_threshold_1_value)

        # Step CV_bitwise_not0:
        __cv_bitwise_not_src1 = hsv_threshold_1_output
        (cv_bitwise_not_output) = __cv_bitwise_not(__cv_bitwise_not_src1)

        # Step Mask0:
        __mask_input = hsv_threshold_0_output
        __mask_mask = cv_bitwise_not_output
        (mask_output) = __mask(__mask_input, __mask_mask)

        # Step Find_Contours0:
        __find_contours_input = mask_output
        (find_contours_output) = __find_contours(__find_contours_input, __find_contours_external_only)

        # Step Filter_Contours0:
        __filter_contours_contours = find_contours_output
        (filter_contours_output) = __filter_contours(__filter_contours_contours, __filter_contours_min_area, __filter_contours_min_perimeter, __filter_contours_min_width, __filter_contours_max_width, __filter_contours_min_height, __filter_contours_max_height, __filter_contours_solidity, __filter_contours_max_vertices, __filter_contours_min_vertices, __filter_contours_min_ratio, __filter_contours_max_ratio)

        # Step Convex_Hulls0:
        __convex_hulls_contours = filter_contours_output
        (convex_hulls_output) = __convex_hulls(__convex_hulls_contours)





        # threshold = threshold_video(lower_green, upper_green, frame)

        rect1 = cv2.rectangle(frame, (0, 300), (640, 480), (0,0,0), -1)
        processedValues = findTargets(rect1, mask_output, vals_to_send, centerX, centerY)

        #if processedValues[3] != None:
            #print(processedValues[3])


        highGoal = {}
        highGoal['y'] = processedValues[0]
        highGoal['z'] = processedValues[1]
        highGoal['yaw'] = processedValues[2]
        if processedValues[3] != None:
            processedValues[3] = abs(round(processedValues[3], 2))
        highGoal['dis'] = processedValues[3]

        highGoal['targid'] = 0

        if processedValues[3] != None:
            print(processedValues[3])
            PacketQueue.put_nowait(highGoal)


    # Tape Process!
    # APPLY A BLUE TO BLUR THE LINES
    else:
        # Ball Tracker!
        original_frame = frame.copy()
        frame = MaskBall(frame)  # Filter and Mask by HSV Values
        Targets = findBalls(frame)

        '''
        #Debug Drawing Code
        for Targ in Targets:
            original_frame = cv2.circle(original_frame, (Targ['x'], Targ['y']), Targ['radius'], (255, 0, 0), 2)
            original_frame = cv2.circle(original_frame, Targ['center'], 4, (0, 0, 255), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            # org
            org = (Targ['x'], Targ['y'])
            # fontScale
            fontScale = .5
            # Blue color in BGR
            color = (255, 0, 0)
            # Line thickness of 2 px
            thickness = 2
            # Using cv2.putText() method
            original_frame = cv2.putText(original_frame, str(Targ['aspectratio']), org, font, fontScale, color, thickness, cv2.LINE_AA)
        cv2.imwrite("of1" + str(val) +  ".jpg", original_frame)
        '''

        someprint = print("end of process frame")

        return processedFrame


# Link to further explanation: https://docs.google.com/presentation/d/1ediRsI-oR3-kwawFJZ34_ZTlQS2SDBLjZasjzZ-eXbQ/pub?start=false&loop=false&slide=id.g12c083cffa_0_298
def calculatePitch(pixelY, centerY, vFocalLength):
    pitch = math.degrees(math.atan((pixelY - centerY) / vFocalLength))
    # Just stopped working have to do this:
    pitch *= -1
    return round(pitch)


def getEllipseRotation(image, cnt):
    try:
        # Gets rotated bounding ellipse of contour
        ellipse = cv2.fitEllipse(cnt)
        centerE = ellipse[0]
        # Gets rotation of ellipse; same as rotation of contour
        rotation = ellipse[2]
        # Gets width and height of rotated ellipse
        widthE = ellipse[1][0]
        heightE = ellipse[1][1]
        # Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
        rotation = translateRotation(rotation, widthE, heightE)

        cv2.ellipse(image, ellipse, (23, 184, 80), 3)
        return rotation
    except:
        # Gets rotated bounding rectangle of contour
        rect = cv2.minAreaRect(cnt)
        # Creates box around that rectangle
        box = cv2.boxPoints(rect)
        # Not exactly sure
        box = np.int0(box)
        # Gets center of rotated rectangle
        center = rect[0]
        # Gets rotation of rectangle; same as rotation of contour
        rotation = rect[2]
        # Gets width and height of rotated rectangle
        width = rect[1][0]
        height = rect[1][1]
        # Maps rotation to (-90 to 90). Makes it easier to tell direction of slant
        rotation = translateRotation(rotation, width, height)
        return rotation


#################### FRC VISION PI Image Specific #############
configFile = "/boot/frc.json"


class CameraConfig: pass


team = None
server = False
cameraConfigs = []

"""Report parse error."""


def parseError(str):
    print("config error in '" + configFile + "': " + str, file=sys.stderr)


"""Read single camera configuration."""


def readCameraConfig(config):
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    cam.config = config

    cameraConfigs.append(cam)
    return True


"""Read configuration file."""


def readConfig():
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    return True


"""Start running the camera."""


def startCamera(config):
    print("Starting camera '{}' on {}".format(config.name, config.path))
    cs = CameraServer.getInstance()
    camera = cs.startAutomaticCapture(name=config.name, path=config.path)
    config.config['pixel format'] = 'yuyv'
    print(config.config)
    camera.setConfigJson(json.dumps(config.config))

    return cs, camera


# TEST FUNCTION for high goal
def ProcessFrameTest(frame):
    source0 = frame

    # Step HSV_Threshold0:
    __hsv_threshold_0_input = source0
    (hsv_threshold_0_output) = __hsv_threshold(__hsv_threshold_0_input, __hsv_threshold_0_hue, __hsv_threshold_0_saturation, __hsv_threshold_0_value)

    # Step HSV_Threshold1:
    __hsv_threshold_1_input = source0
    (hsv_threshold_1_output) = __hsv_threshold(__hsv_threshold_1_input, __hsv_threshold_1_hue, __hsv_threshold_1_saturation, __hsv_threshold_1_value)

    # Step CV_bitwise_not0:
    __cv_bitwise_not_src1 = hsv_threshold_1_output
    (cv_bitwise_not_output) = __cv_bitwise_not(__cv_bitwise_not_src1)

    # Step Mask0:
    __mask_input = hsv_threshold_0_output
    __mask_mask = cv_bitwise_not_output
    (mask_output) = __mask(__mask_input, __mask_mask)

    # Step Find_Contours0:
    __find_contours_input = mask_output
    (find_contours_output) = __find_contours(__find_contours_input, __find_contours_external_only)

    # Step Filter_Contours0:
    __filter_contours_contours = find_contours_output
    (filter_contours_output) = __filter_contours(__filter_contours_contours, __filter_contours_min_area, __filter_contours_min_perimeter, __filter_contours_min_width, __filter_contours_max_width, __filter_contours_min_height, __filter_contours_max_height, __filter_contours_solidity, __filter_contours_max_vertices, __filter_contours_min_vertices, __filter_contours_min_ratio, __filter_contours_max_ratio)

    # Step Convex_Hulls0:
    __convex_hulls_contours = filter_contours_output
    (convex_hulls_output) = __convex_hulls(__convex_hulls_contours)

    rect1 = cv2.rectangle(frame, (0, 300), (640, 480), (0,0,0), -1)
    processedValues = findTargets(rect1, mask_output, vals_to_send, centerX, centerY)

    highGoal = {}
    highGoal['y'] = processedValues[0]
    highGoal['z'] = processedValues[1]
    highGoal['yaw'] = processedValues[2]
    if processedValues[3] != None:
        processedValues[3] = abs(round(processedValues[3], 2))
    highGoal['dis'] = processedValues[3]
    highGoal['targid'] = 0
    return highGoal



# Check if Test Playback Mode
def isTestMode(value):
    if(str(value).strip() == "test"):
        return True 
    return False 

# reads jpgs from directory
def runTestMode(folder):
    # read all images 
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    
    print("Found " + str(len(images)) + " Images")

    for img in images:
        processedInfo = ProcessFrameTest(img)



if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]

        # check for test mode!
        if(isTestMode(sys.argv[1])):
            print("Test Mode Detected")
            if(len(sys.argv) > 2):
                # Has a Directories to process
                targetDirectory = sys.argv[2]
                runTestMode(targetDirectory)
            else:
                print("No Directory Found!")


            exit()


    # read configuration
    if not readConfig():
        sys.exit(1)


    # Normal Operation
    from cscore import CameraServer, VideoSource

    # start cameras
    cameras = []
    streams = []
    for cameraConfig in cameraConfigs:
        cs, cameraCapture = startCamera(cameraConfig)
        streams.append(cs)
        cameras.append(cameraCapture)

    webcam = cameras[0]
    cameraServer = streams[0]
    # Start thread reading camera
    cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()

    # Create Socket Thread and Pass the Socket with Packet Queue
    SocketThread = SocketWorker(PacketQueue).start()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(image_height, image_width, 3), dtype=np.uint8)

    # cap.autoExpose=True;
    tape = False
    # fps = FPS().start()
    # TOTAL_FRAMES = 200;
    # loop forever
    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        timestamp, img = cap.read()

        Tape = True
        frame = ProcessFrame(img, Tape)

    # Doesn't do anything at the moment. You can easily get this working by indenting these three lines
    # and setting while loop to: while fps._numFrames < TOTAL_FRAMES
    # fps.stop()
    print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
    print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))