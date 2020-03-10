#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.

# My 2019 license: use it as much as you want. Crediting is recommended because it lets me know that I am being useful.
# Credit to Screaming Chickens 3997

# This is meant to be used in conjuction with WPILib Raspberry Pi image: https://github.com/wpilibsuite/FRCVision-pi-gen
#----------------------------------------------------------------------------

import json
import time
import sys
from threading import Thread

from cscore import CameraServer, VideoSource, VideoMode, UsbCamera
from networktables import NetworkTablesInstance
import cv2
import numpy as np
from networktables import NetworkTables
import math
from grip_target_detection_v_one import GripPipeline


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

        #Automatically sets exposure to 0 to track tape
        self.webcam = camera
        #Make a blank image to write on
        self.img = np.zeros(shape=(frameWidth, frameHeight, 3), dtype=np.uint8)
        #Gets the video
        self.stream = cameraServer.getVideo(camera = camera)
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
            #gets the image and timestamp from cameraserver
            (self.timestamp, self.img) = self.stream.grabFrame(self.img)

    def read(self):
        # return the frame most recently read
        return self.timestamp, self.img

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
    def getError(self):
        return self.stream.getError()



image_height = 240
image_width = 320
team = 587

def calculateYaw(pixelX, centerX, hFocalLength):
    yaw = math.degrees(math.atan((pixelX - centerX) / hFocalLength))
    return round(yaw)


def extra_processing(pipeline, img):
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
    table = NetworkTables.getTable('VisionTarget')
    print('Number of Contours : ', pipeline.filter_contours_output.__len__())
    table.putNumber('numContours', pipeline.filter_contours_output.__len__())
    
    targetFound = False
    distance = 0
    targetAngle = 0
    largestArea = 0
    for contour in pipeline.filter_contours_output:
        #returns a Box2D structure which contains following detals
        #( top-left corner(x,y), (width, height), angle of rotation )
        point, dimensions, angle = cv2.minAreaRect(contour)
        x, y = point
        w, h = dimensions
        #print(angle)
        center_x_positions.append(x + w / 2)  # X and Y are coordinates of the top-left corner of the bounding box
        center_y_positions.append(y + h / 2)
        widths.append(w)
        heights.append(h)

        area = w*h
        ratio = w/h
        flip = 0
        # make sure contour is fairly level either close to 0 degrees or 90 if rotated
        if abs(angle) > 10:
            # if rotate switch the width and height
            if (abs(angle) > 80) and (abs(angle) < 100):
                ratio = h/w
                flip = 1
            else:
                continue

        # make sure width is a bit wider than height, if not probably not a target
        if (ratio < 1.3) or (ratio > 2.7):
            print('w ', w, ' h ', h, ' angle: ', angle, ', ratio: ', ratio)
            continue

        cv2.circle(img, (int(x),int(y)), int((w+h)/2), (255,0,0), 1)
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.1 * peri, True)

        if area > largestArea:
            largestArea = area
            #distance is in inches
            distance = (17 * 240) / (2 * h * math.tan(10.27*math.pi/180))
            # check to see if the bounding box is 90 degrees off, if so use width, since that is really the height
            if flip == 1:
                distance = (17 * 240) / (2 * w * math.tan(10.27*math.pi/180))

            targetAngle = math.atan((x - 160)/640)
            #targetAngle = 14 * ((x - 160) / 160)
            targetFound = True
            # Elias (distance to target = target length in pixels/1.7333,  degrees to turn inverse tan(Pxoff/640px))
            #target_x
            #target_y
            print('(', x,',', y, '), ', w, ' X ', h, ' angle: ', angle, ', vertex: ', len(approx), ', distance: ', distance, 'angle: ', targetAngle*180/3.141592)

    # Publish to the '/vision/red_areas' network table



    table.putNumber('distance', distance)
    table.putNumber('targetAngle', targetAngle)
    
    table.putBoolean('targetFound', targetFound)

    #table.putNumberArray('x', center_x_positions)
    #table.putNumberArray('y', center_y_positions)
    #table.putNumberArray('width', widths)
    #table.putNumberArray('height', heights)
 


def startCamera():
    print("Starting camera rPi on /dev/video0")
    cs = CameraServer.getInstance()
    camera = UsbCamera("rPi Camera", "/dev/video0")
    #camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 60)
    camera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 60)
    #camera = cs.startAutomaticCapture(name="rPi Camera", path="/dev/video0")
    server = cs.startAutomaticCapture(camera=camera, return_server=True)
    server.setCompression(25)
    server.setFPS(45)
    #camera.setResolution(320,240)
    #camera.setConfigJson(json.dumps(config.config))
    camera.getProperty('color_effects').set(3)
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)
    return cs, camera

if __name__ == "__main__":

    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    #Name of network table - this is how it communicates with robot. IMPORTANT
    networkTable = NetworkTables.getTable('Vision')

    print("Setting up NetworkTables client for team {}".format(team))
    ntinst.startClientTeam(team)

    # start camera
    cameraServer, webcam = startCamera()

    #Start thread reading camera
    cap = WebcamVideoStream(webcam, cameraServer, image_width, image_height).start()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(image_height, image_width, 3), dtype=np.uint8)
    #Start thread outputing stream
    #streamViewer = VideoShow(image_width,image_height, cameraServer, frame=img, name="Vision").start()

    outstream = cameraServer.putVideo("Processed", 320, 240)
    outstream.setFPS(10)

    print('Creating pipeline')
    pipeline = GripPipeline()

    # loop forever
    while True:

        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        timestamp, img = cap.read()

        if timestamp == 0:
            # Send the output the error.
            #streamViewer.notifyError(cap.getError());
            # skip the rest of the current iteration
            continue

        pipeline.process(img)
        extra_processing(pipeline, img)

        #Puts timestamp of camera on netowrk tables
        networkTable.putNumber("VideoTimestamp", timestamp)
        #streamViewer.frame = processed
        #Flushes camera values to reduce latency
        ntinst.flush()

        outstream.putFrame(img)
 
    