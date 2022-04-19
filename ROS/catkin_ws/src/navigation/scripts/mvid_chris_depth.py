#!/usr/bin/env python3

import numpy as np
import numpy.ma as ma
from itertools import groupby
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

def get_longest_false_run(list):
    start = 0
    runs = []
    for key, run in groupby(list):
        length = sum(1 for _ in run)
        runs.append((start, start + length -1))
        start += length
    result = max(runs, key=lambda x: x[1] - x[0])
    print(result)
    if result[1] - result[0] < chair_pixels:
        target_index = -1
    else:
        ##find center position by using simple average
        target_index = (result[0] + result[1])/2
    return target_index
    
def get_arc_and_ppm(range_of_concern):
    range_arc_length = 1.22 * range_of_concern
    ##chair arc can fit 1.53 * range of concern 
    ## Pixels per meter (ppm) = 512/arc length in meters
    ppm = 512 / range_arc_length
    ## ppm * .82 = chair number of pixels necessary for space
    chair_pixels = .82 * ppm
    return range_arc_length, ppm, chair_pixels


def get_boolean_with_np(arr):
    return list(np.sum((arr[0:-1,250:300] < range_of_concern),axis=1, dtype=bool))

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

while 1:
    if listener.hasNewFrame():
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

    # #get kinect input__________________________________________________________________________
            # dst = pretty_depth(cv2.resize(depth.asarray(),(int(512), int(428))))
            depth = (depth.asarray()).astype(uint16)
            depth = depth.reshape(424,512)
            dst = depth
            # cv2.imshow("Depthvtr56432", dst)
            
            classIds, confs, bbox = net.detect(cv2.cvtColor(color.asarray(), cv2.COLOR_RGB2BGR), confThreshold = thres)
            bbox = list(bbox)
            confs = list(np.array(confs).reshape(1,-1)[0])
            confs = list(map(float,confs))
            indices = cv2.dnn.NMSBoxes(bbox,confs,thres,nms_threshold)
            masked_depth = np.ma.masked_equal(depth, 0, copy=False)
            print('indices', indices )
            print('bbox', bbox )
            for i in indices:
                try:
                    box = bbox[i]
                    x,y,w,h = box[0], box[1], box[2], box[3]
                    cv2.rectangle(color.asarray(), (x,y), (x+w,y+h), color = (0,255,0), thickness=3)
                except IndexError as e:
                    print("IndexError: ", e)
                # except ValueError as e:
                #     print("ValueError: ", e)
            new_img = color.asarray()
            try:
                x1  = bbox[0][0]
                x2  = bbox[0][0] + bbox[0][2]
                y1  = bbox[0][1]
                y2  = bbox[0][1] + bbox[0][3]
                print(x1,x2,y1,y2)
                new_img[:y1,:] = 0
                new_img[y2:,:] = 0
                new_img[:,:x1] = 0
                new_img[:,x2:] = 0
                # print("sliced rgb")
                # xd1 = x1/4
                # xd2 = x2/4
                # yd1 = y1/4
                # yd2 = y2/4
                # new_img_depth = depth
                # new_img_depth[:y1,:] = 0
                # new_img_depth[y2:,:] = 0
                # new_img_depth[:,:x1] = 0
                # new_img_depth[:,x2:] = 0
                # print("Finished slicing depth image")
            except Exception as e:
                print("Exception: ", e)
            cv2.imshow("new_img", cv2.resize(new_img, (int(800), int(600))))
 
    # #rectangular border (improved edge detection + closed contours)___________________________ 
            cv2.rectangle(dst,(0,0),(1920,1080),(40,100,0),2)
           
    # #defined points approach #                 
            spac = 30
            (rows,cols)=dst.shape
            # print(dst.shape)
            shared = str("False")
        except AttributeError as e:
            print("Error no Object:", e)
        counter = 0
        range_of_concern = 6
        range_arc_length, ppm, chair_pixels = get_arc_and_ppm(range_of_concern)
        depth_vision = [0]*512

        depth_vision = get_boolean_with_np(new_img_depth)
        # print(depth_vision)
        # with open("/home/josh/Documents/depth.csv","w") as f:
        #     np.savetxt(f,depth_vision)
        ##depth vision is a 1x512 boolean list. need to identify which is best place to go
        
        target_index = int(get_longest_false_run(depth_vision))


        with open("/home/josh/Documents/share.json","w") as f:
            kinect_dict["target"] = target_index
            kinect_dict["range"] = range_of_concern
            json.dump(kinect_dict, f)
        print("Range of Concern: ", range_of_concern)
        print("target index: ", target_index)
        target_offset = (target_index - 256)/ ppm
        print("Target offset: ", target_offset)
        depth[250:300,target_index-1:target_index+1] = 32168
        cv2.imshow("depth", depth)

        # cv2.imshow('Video', dst)

        listener.release(frames)
            
        key = cv2.waitKey(delay=1)
        if key == ord('q'):
            break

    # time.sleep(2)
        
device.stop()
