# -*- coding: utf-8 -*-
"""
Created on Sat Jan 19 13:10:58 2019
@author: Albert Lin
"""

#!/usr/bin/python3

"""
taken from GRIP repository
Sample program that uses a generated GRIP pipeline to detect lines in an image and publish important features to NetworkTables.
"""

import cv2
import math
from networktables import NetworkTables
from grip_target_detection_v_one import GripPipeline
from Threads import CameraThread
from cscore import CameraServer, VideoSource, VideoCamera, MjpegServer
import numpy as np


def calculateYaw(pixelX, centerX, hFocalLength):
    yaw = math.degrees(math.atan((pixelX - centerX) / hFocalLength))
    return round(yaw)



def extra_processing(pipeline):
    """
    Performs extra processing on the pipeline's outputs and publishes data to NetworkTables.
    :param pipeline: the pipeline that just processed an image
    :return: None
    """
    #Tape 39 inches wide, 17 inches high
    #43.5 inches tall at 120 inches away (20.5 degrees)

    center_x_positions = []
    center_y_positions = []
    widths = []
    heights = []

    # Tartget coordinate should be ((x + w/2), h)

    print('Number of Contours : ', pipeline.filter_contours_output.__len__())

    targetFound = False
    distance = 0
    targetAngle = 0
    for contour in pipeline.filter_contours_output:
        #returns a Box2D structure which contains following detals
        #( top-left corner(x,y), (width, height), angle of rotation )
        point, dimensions, angle = cv2.minAreaRect(contour)
        x, y = point
        w, h = dimensions
        print(angle)
        center_x_positions.append(x + w / 2)  # X and Y are coordinates of the top-left corner of the bounding box
        center_y_positions.append(y + h / 2)
        widths.append(w)
        heights.append(h)
        
        #distance is in inches
        distance = (39 * 320) / (2 * w * math.tan(14*math.pi/180))

        targetAngle = math.atan(((x + w / 2) - 160)/640)
        targetFound = True
        # Elias (distance to target = target length in pixels/1.7333,  degrees to turn inverse tan(Pxoff/640px))
        #target_x
        #target_y
        print('distance : ', distance)
        print('angle    : ', targetAngle)

    # Publish to the '/vision/red_areas' network table

    table = NetworkTables.getTable('VisionTarget')

    table.putNumber('distance', distance)
    table.putNumber('targetAngle', targetAngle)
    table.putBoolean('targetFound', targetFound)
    #table.putNumberArray('x', center_x_positions)
    #table.putNumberArray('y', center_y_positions)
    #table.putNumberArray('width', widths)
    #table.putNumberArray('height', heights)
    


def main():
    print('Initializing NetworkTables')
    #NetworkTables.setClientMode()
    #NetworkTables.setIPAddress('localhost')
    NetworkTables.initialize()
    
    print('Starting camera input thread')
    #cameraThread = CameraThread()
    #cameraThread.start()
    
    
    inst = CameraServer.getInstance()
    #inst.enableLogging()

    #E = inst.addServer("OurVideo")
    #E.setSource(inst)

    #cap = cv2.VideoCapture('/dev/video0')

    camera = inst.startAutomaticCapture(name = "Cam0", path = '/dev/video0')

    #cvSink = inst.getVideo()

    outputStream = inst.putVideo("Vision", 320, 240)

    #img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)


    print('Creating pipeline')
    pipeline = GripPipeline()

    print('Running pipeline')
    while True:
        have_frame, frame = cameraThread.read()
        # Display the resulting frame
        #cv2.imshow('Frame',frame)
        # cv2.waitKey(25)

        #time, img = cvSink.grabFrame(img)

        #ret, frame = cap.read()

        #if time == 0:

        #    outputStream.notifyError(cvSink.getError());

        #    continue

        if have_frame:
            pipeline.process(frame)
            extra_processing(pipeline)
            outputStream.putFrame(frame)

    print('Capture closed')



if __name__ == '__main__':
    main()