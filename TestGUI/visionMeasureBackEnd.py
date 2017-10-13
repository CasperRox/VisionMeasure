try:
    import tkinter as tk  # for python 3
except:
    import Tkinter as tk  # for python 2

import pygubu
import numpy as np
import cv2
import math
import sys
from multiprocessing import Queue

upperBoundary = (125, 240, 200)
lowerBoundary = (110, 200, 100)

q = Queue()
qSum = 0

def liveVideo():
    print (upperBoundary)
    print (lowerBoundary)

    cap = cv2.VideoCapture(1)
    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()
        if ret:
            # cv2.imshow("Original", frame)
            blobDistance(frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def setRed():
    global upperBoundary, lowerBoundary
    upperBoundary = (185, 255, 255)
    lowerBoundary = (155, 50, 50)

def setGreen():
    global upperBoundary, lowerBoundary
    upperBoundary = (90, 150, 120)
    lowerBoundary = (75, 100, 70)

def setYellow():
    global upperBoundary, lowerBoundary
    upperBoundary = (35, 180, 255)
    lowerBoundary = (20, 80, 180)

def setPink():
    global upperBoundary, lowerBoundary
    upperBoundary = (125, 240, 200)
    lowerBoundary = (110, 200, 100)

def getmmDistance(pixel):
    temp = ((pixel - 23.051) / 0.9753)-1
    if temp > 0:
        return temp
    else:
        return 0

def blobDistance(imgSrc):
    global upperBoundary, lowerBoundary
    global q, qSum
    imgOrig = imgSrc
    blurred = cv2.GaussianBlur(imgSrc, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color, then perform a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, lowerBoundary, upperBoundary)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the edge map
    cntsPre = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts = []

    # filtering the list of contours according to contour area
    if len(cntsPre) > 0:
        for i in range(0,len(cntsPre)):
            # print ("AreaPre : ", cv2.contourArea(cntsPre[i]))
            if 20 <= cv2.contourArea(cntsPre[i]) <= 100 :
                cnts.append(cntsPre[i])

    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:2]
    center = [(0,0),(0,0)]

    # Draw a rectangle on the image frame
    imgTemp = imgOrig.copy()
    cv2.rectangle(imgTemp,(0,0),(imgTemp.shape[1],100),(0,0,0),-1)
    cv2.addWeighted(imgTemp,0.5,imgOrig,0.5,0,imgOrig)
    cv2.putText(imgOrig, "Press 'q' or 'Ctrl+C' to Exit", (390,20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

    # only proceed if two contours were found
    if len(cnts) > 1:
        for i in range(0,len(cnts)):
            # Compute the minimum enclosing circle and centroid
            ((x, y), radius) = cv2.minEnclosingCircle(cnts[i])
            M = cv2.moments(cnts[i])
            center[i] = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            # print (center[i])
            cv2.circle(imgOrig, (int(x), int(y)), int(radius),(0, 255, 255), 2)
            cv2.circle(imgOrig, center[i], 5, (0, 0, 255), -1)
        distance = math.sqrt(math.pow((center[0][1]-center[1][1]),2) + math.pow((center[0][0]-center[1][0]),2))

        if q.qsize() >= 5:
            qAvg = qSum / q.qsize()
            # print ("Sum : ", qSum)
            # print ("Avg : ", qAvg)
            qSum -= q.get()

            # show distance value on image frame
            font = cv2.FONT_HERSHEY_SCRIPT_COMPLEX
            # cv2.putText(imgOrig, '%d mm' %getmmDistance(distance), (10,80), font, 3, (0,0,255), 7, cv2.LINE_AA)
            cv2.putText(imgOrig, '%.1f cm' %(getmmDistance(qAvg)/10), (10,80), font, 3, (0,0,255), 7, cv2.LINE_AA)

            print ("Pixel Distance : ", qAvg)
            print ("mm Distance : ", "%.2f" %getmmDistance(qAvg))

        qSum += distance
        q.put(distance)

    imgOrig = cv2.resize(imgOrig, (1350,730))
    cv2.imshow("VisionMeasure", imgOrig)
    # cv2.imshow("Filtered", mask)

class Application:
    def __init__(self, master):

        #1: Create a builder
        self.builder = builder = pygubu.Builder()

        #2: Load an ui file
        builder.add_from_file('visionMeasureFrontEnd.ui')

        #3: Create the widget using a master as parent
        self.mainwindow = builder.get_object('Toplevel_1', master)

        # Configure callbacks
        callbacks = {
            'showVideo': liveVideo,
            'thresholdRed': setRed,
            'thresholdGreen': setGreen,
            'thresholdYellow': setYellow,
            'thresholdPink': setPink
        }
        builder.connect_callbacks(callbacks)


if __name__ == '__main__':
    root = tk.Tk()
    # root.withdraw()
    app = Application(root)
    root.mainloop()

