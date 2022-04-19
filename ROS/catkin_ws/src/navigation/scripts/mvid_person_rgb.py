#!/usr/bin/env python3

import numpy as np
import numpy.ma as ma
from itertools import groupby
import cv2
import scipy.misc
import signal
from numpy import testing, uint16
import json
from functions import *
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

try:
    from pylibfreenect2 import OpenGLPacketPipeline
    pipeline = OpenGLPacketPipeline()
except:
    from pylibfreenect2 import CpuPacketPipeline
    pipeline = CpuPacketPipeline()

def sigint_handler(signum, frame):
    print("Got SIGINT, shutting down...")
    quit()

def nothing(x):
    pass

def pretty_depth(depth):
    depth = depth.astype(np.uint8)
    return depth

def get_closest(depthData):
    global range_of_concern
    distance = 4000
    while distance > 1000:
        ret,objectMask = cv2.threshold(depthData, distance, 1, cv2.THRESH_BINARY_INV)
        pixelCount = round(np.average(objectMask))
        if pixelCount < 1:
            print("breaking")
            break
        distance = distance - 400
    # print("Done with image")
    with open("/home/josh/Documents/depth.csv","w") as f:
        np.savetxt(f,objectMask)
    print("Distance at finish: ", distance)
    range_of_concern = distance*0.001

signal.signal(signal.SIGINT, sigint_handler)

fn = Freenect2()
num_devices = fn.enumerateDevices()
serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

registration = Registration(device.getIrCameraParams(),device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)
device.start()

bigdepth = Frame(1920, 1082, 4)
color_depth_map = np.zeros((424, 512),  np.int32).ravel() 

kernel = np.ones((5, 5), np.uint8)

thres = 0.5
nms_threshold = 0.6

classFile = 'coco.names'
classNames = []
with open(classFile,'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320,320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5,127.5,127.5))
net.setInputSwapRB(True)
kinect_dict = {"target" : 256, "range_of_concern" : 0}
range_of_concern = 1.5

while 1:
    if listener.hasNewFrame():
        avoidProtocol = False
        x, x2, boxSize = []
        frames = listener.waitForNewFrame()
        color = frames["color"]
        ir = frames["ir"]
        depth = frames["depth"]
        registration.apply(color, depth, undistorted, registered,bigdepth=bigdepth,color_depth_map=color_depth_map)
        classIds, confs, bbox = net.detect(cv2.cvtColor(color.asarray(),cv2.COLOR_RGBA2RGB),confThreshold = thres)

        try:
            for classId, confidence, box in zip(classIds,confs,bbox):
                cv2.rectangle(color.asarray(), box, color = (0,255,0), thickness=3)
                cv2.putText(color.asarray(), classNames[classId-1].upper(), (box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 2)
                cv2.putText(color.asarray(), str(round(confidence*100,3)) + "%", (box[0]+10,box[1]+70), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 2)
                if classNames[classId-1].upper() == "PERSON":
                    boxSize.append(box[0]*box[1]) #multiply y * x
                    x.append(box[0])
                    x2.append(box[0]+box[3])
    # #get kinect input__________________________________________________________________________
            # dst = pretty_depth(cv2.resize(depth.asarray(),(int(512), int(428))))
            depth = (depth.asarray()).astype(uint16)
            depth = depth.reshape(424,512)
            dst = depth
            
            classIds, confs, bbox = net.detect(cv2.cvtColor(color.asarray(), cv2.COLOR_RGB2BGR), confThreshold = thres)
            bbox = list(bbox)
            confs = list(np.array(confs).reshape(1,-1)[0])
            confs = list(map(float,confs))
            indices = cv2.dnn.NMSBoxes(bbox,confs,thres,nms_threshold)
            masked_depth = np.ma.masked_equal(depth, 0, copy=False)
            cv2.imshow("RGB", cv2.resize(color.asarray(),(int(800), int(600))))
 
    # #rectangular border (improved edge detection + closed contours)___________________________ 
            cv2.rectangle(dst,(0,0),(1920,1080),(40,100,0),2)
           
    # #defined points approach #                 
            spac = 30
            (rows,cols)=color.shape
            # print(dst.shape)
            shared = str("False")
        except AttributeError as e:
            print("Error no Object:", e)
        counter = 0


####### New Navigational Development with Kinect Depth Measurements #####        
        range_arc_length = 1.22 * range_of_concern
        ppm = cols / range_arc_length
        chair_pixels = .82 * ppm
        depth_vision = [0] * 1920

        for person, l, r in boxSize, x, x2:
            if person > 500000:
                depth_vision[l:r] = 1

        start = 0
        runs = []
        for key, run in groupby(depth_vision):
            length = sum(1 for _ in run)
            runs.append((start, start + length -1))
            start += length
        result = max(runs, key=lambda x: x[1] - x[0])
        if result[1] - result[0] < chair_pixels:
            target_index = -1
        else:
            ##find center position by using simple average
            target_index = (result[0] + result[1])/2
        
        with open("/home/josh/Documents/share.json","w") as f:
            kinect_dict["target"] = target_index
            kinect_dict["range"] = range_of_concern
            json.dump(kinect_dict, f)
        print("target index: ", target_index)
        target_offset = target_index - 960 / ppm
        print("Target offset: ", target_offset)


        cv2.imshow('Video', dst)

        listener.release(frames)
            
        key = cv2.waitKey(delay=1)
        if key == ord('q'):
            break
        
device.stop()
