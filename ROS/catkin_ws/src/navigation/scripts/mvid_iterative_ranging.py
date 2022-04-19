#!/usr/bin/env python3

import numpy as np
import numpy.ma as ma
from itertools import groupby
from operator import itemgetter
import cv2
import scipy.misc
import signal
import time
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

def longest_false_run(lst):
    """Finds the longest false run in a list of boolean"""

    # find False runs only
    groups = [[i for i, _ in group] for key, group in groupby(enumerate(lst), key=itemgetter(1)) if not key]

    # get the one of maximum length
    group = max(groups, key=len, default=[-1, -1])

    start, end = group[0], group[-1]

    return start, end

    
def get_arc_and_ppm(range_of_concern):
    range_arc_length = 1.22 * range_of_concern
    ##chair arc can fit 1.53 * range of concern 
    ## Pixels per meter (ppm) = 512/arc length in meters
    ppm = 512 / range_arc_length
    ## ppm * .82 = chair number of pixels necessary for space
    chair_pixels = .74 * ppm
    return range_arc_length, ppm, chair_pixels


def get_boolean_with_np(arr):
    with open("/home/josh/Documents/arr.csv","w") as f:
        np.savetxt(f,arr, fmt="%d", delimiter=',')
    return list(np.sum((arr[120:180,0:-1] < range_of_concern),axis=0, dtype=bool))

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
    with open("/home/josh/Documents/objectMask.csv","w") as f:
        np.savetxt(f,objectMask, fmt="%d", delimiter=',')
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
range_of_concern = 1

while 1:
    if listener.hasNewFrame():
        frames = listener.waitForNewFrame()
        color = frames["color"]
        ir = frames["ir"]
        depth = frames["depth"]
        registration.apply(color, depth, undistorted, registered,bigdepth=bigdepth,color_depth_map=color_depth_map)
        classIds, confs, bbox = net.detect(cv2.cvtColor(color.asarray(),cv2.COLOR_RGBA2RGB),confThreshold = thres)
        # color = np.fliplr(color.asarray())
        try:			
            for classId, confidence, box in zip(classIds,confs,bbox):
                cv2.rectangle(color.asarray(), box, color = (0,255,0), thickness=3)
                cv2.putText(color.asarray(), classNames[classId-1].upper(), (box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 2)
                cv2.putText(color.asarray(), str(round(confidence*100,3)) + "%", (box[0]+10,box[1]+70), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 2)

    # #get kinect input__________________________________________________________________________
            # dst = pretty_depth(cv2.resize(depth.asarray(),(int(512), int(428))))
            depth = (depth.asarray()).astype(uint16)
            depth = depth.reshape(424,512)
            dst = depth

            with open("/home/josh/Documents/depth.csv","w") as f:
                np.savetxt(f,depth, fmt="%d", delimiter=",")
            
            classIds, confs, bbox = net.detect(cv2.cvtColor(color.asarray(), cv2.COLOR_RGB2BGR), confThreshold = thres)
            bbox = list(bbox)
            confs = list(np.array(confs).reshape(1,-1)[0])
            confs = list(map(float,confs))
            depth = depth[100:-1, 0:-1]
            new_depth_img = np.fliplr(depth)
            with open("/home/josh/Documents/new_depth_image.csv","w") as f:
                np.savetxt(f,new_depth_img, fmt="%d", delimiter=",")
            get_closest(new_depth_img)

            cv2.imshow("color image", cv2.resize(color.asarray(), (int(800), int(600))))
    # #rectangular border (improved edge detection + closed contours)___________________________ 
            # cv2.rectangle(dst,(0,0),(1920,1080),(40,100,0),2)
           
    # #defined points approach #                 
            spac = 30
            (rows,cols)=dst.shape
            # print(dst.shape)
            shared = str("False")
        except AttributeError as e:
            print("Error no Object:", e)
        counter = 0
        if range_of_concern == None:
            range_of_concern = 1
        range_arc_length, ppm, chair_pixels = get_arc_and_ppm(range_of_concern)

        depth_vision = get_boolean_with_np(new_depth_img)
        # print(depth_vision)
        ##depth vision is a 1x512 boolean list. need to identify which is best place to go
        
        start, end = longest_false_run(depth_vision)
        print(start, end)
        
        with open("/home/josh/Documents/depth_vision.csv","w") as f:
            np.savetxt(f,depth_vision, fmt="%d", delimiter=",")
            f.write(str(start)+ ',' + str(end))

        if end - start < chair_pixels:
            target_index = -1
        else:
            ##find center position by using simple average
            target_index = int((start+end) / 2)

        with open("/home/josh/Documents/share.json","w") as f:
            kinect_dict["target"] = target_index
            kinect_dict["range"] = range_of_concern
            json.dump(kinect_dict, f)
        print("Range of Concern: ", range_of_concern)
        print("target index: ", target_index)
        target_offset = (target_index - 256)/ ppm
        print("Target offset: ", target_offset)
        if target_index > 0:
            new_depth_img[135:215, start-1:start+1] = 32168
            new_depth_img[150:200,target_index-1:target_index+1] = 32168
            new_depth_img[135:215, end-1:end+1] = 32168

        cv2.imshow("new_depth_image", new_depth_img)
        # cv2.line(new_depth_img, (target_index, 250), (target_index, 300),color=(255,255,255), thickness=6)
        # cv2.imshow('Video', dst)

        listener.release(frames)
            
        key = cv2.waitKey(delay=1)
        if key == ord('q'):
            break

    # time.sleep(2)

with open("/home/josh/Documents/share.json","w") as f:
        kinect_dict["target"] = 256
        kinect_dict["range"] = 0
        json.dump(kinect_dict, f)
device.stop()
