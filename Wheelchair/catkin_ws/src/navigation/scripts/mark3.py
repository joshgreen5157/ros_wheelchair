#!/usr/bin/env python3

import numpy as np
import cv2
import scipy.misc
import signal
import pyfreenect2
from numpy import testing, uint16
import pickle
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
    # np.clip(depth, 0, 2**10 - 1, depth)
    # depth >>= 2
    depth = depth.astype(np.uint8)
    return depth


# def setupComPort(comPort):
#     serialPort = serial.Serial(port = comPort, baudrate = 9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
#     return serialPort

# COM = setupComPort("/dev/ttyACM0")

# def writeCommand(comPort, strvar):
#     comPort.write(str.encode(strvar + '*'))    

# def translate_commands(target):
#     global COM
#     lineA = int(target.linear.x)
#     lineB = int(target.angular.z)
    
#     if lineA> 0:
#         lineA = lineA+170
#     elif lineA< 0:
#         lineA = lineA+100
#     elif lineA == 0:
#         lineA = 130
#     if lineB> 0:
#         lineB = lineB+170
#     elif lineB< 0:
#         lineB = lineB+100
#     elif lineB == 0:
#         lineB = lineB+130
#     lineA = 'A' + str(lineA)
#     lineB = 'B' + str(lineB)
#     print('x = ',target.linear.x,'a = ', lineA)
#     print('y = ',target.angular.z,'b = ', lineB)
#     writeCommand(COM, lineA)
#     writeCommand(COM, lineB)

signal.signal(signal.SIGINT, sigint_handler)

fn = Freenect2()
num_devices = fn.enumerateDevices()
serial = fn.getDeviceSerialNumber(0)
device = fn.openDevice(serial, pipeline=pipeline)

# frameListener = pyfreenect2.SyncMultiFrameListener(pyfreenect2.Frame.COLOR, pyfreenect2.Frame.IR,pyfreenect2.Frame.DEPTH)

registration = Registration(device.getIrCameraParams(),device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)
device.start()

bigdepth = Frame(1920, 1082, 4)
color_depth_map = np.zeros((424, 512),  np.int32).ravel() 

# classFile = 'coco.names'
# classNames = []
# with open(classFile,'rt') as f:
#     classNames = f.read().rstrip('\n').split('\n')

# configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
# weightsPath = 'frozen_inference_graph.pb'

# net = cv2.dnn_DetectionModel(weightsPath,configPath)
# net.setInputSize(320,320)
# net.setInputScale(1.0 / 127.5)
# net.setInputMean((127.5,127.5,127.5))
# net.setInputSwapRB(True)

# cv2.namedWindow('Video',cv2.WINDOW_AUTOSIZE)
# cv2.moveWindow('Video',0,0)
# cv2.resizeWindow('Video',400,100)
# cv2.namedWindow('Navig',cv2.WINDOW_AUTOSIZE)
# cv2.resizeWindow('Navig',400,100)
# cv2.moveWindow('Navig',700,0)
kernel = np.ones((5, 5), np.uint8)

# print('Press \'b\' in window to stop')
# cv2.createTrackbar('val1', 'Video', 37, 1000, nothing)
# cv2.createTrackbar('val2', 'Video', 43, 1000, nothing)
# cv2.createTrackbar('bin', 'Video',20,50,nothing)

thres = 0.5

while 1:
	if listener.hasNewFrame():
		frames = listener.waitForNewFrame()
		color = frames["color"]
		ir = frames["ir"]
		depth = frames["depth"]
		registration.apply(color, depth, undistorted, registered,bigdepth=bigdepth,color_depth_map=color_depth_map)

		# classIds, confs, bbox = net.detect(cv2.cvtColor(color.asarray(),cv2.COLOR_RGBA2RGB),confThreshold = thres)
		# for classId, confidence, box in zip(classIds.flatten(),confs.flatten(),bbox):
		# 	cv2.rectangle(color.asarray(), box, color = (0,255,0), thickness=3)
		# 	cv2.putText(color.asarray(), classNames[classId-1].upper(), (box[0]+10,box[1]+30), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 2)
		# 	cv2.putText(color.asarray(), str(round(confidence*100,3)) + "%", (box[0]+10,box[1]+70), cv2.FONT_HERSHEY_COMPLEX, 2, (0,255,0), 2)

		flag120=[1, 1, 1, 1]
		flag140=[1, 1, 1, 1]
		f14=0
		f12=0
		f10=0
		f8=0

# #get kinect input__________________________________________________________________________
		# dst = pretty_depth(cv2.resize(depth.asarray(),(int(512), int(428))))
		depth = (depth.asarray()).astype(uint16)
		depth = depth.reshape(424,512)
		dst = depth
		cv2.imshow("Depth", dst)

# #rectangular border (improved edge detection + closed contours)___________________________ 
		cv2.rectangle(dst,(0,0),(1920,1080),(40,100,0),2)
	   
# #image binning (for distinct edges)________________________________________________________
		# binn=cv2.getTrackbarPos('bin', 'Video') 
		# e=cv2.getTrackbarPos('erode', 'Video') 
		# dst = (dst/binn)*binn
		# dst = cv2.erode(dst, kernel, iterations=e)
		
# #Video detection___________________________________________________________________________
		# v1 = cv2.getTrackbarPos('val1', 'Video')
		# v2 = cv2.getTrackbarPos('val2', 'Video')
		# edges = cv2.Canny(dst, v1, v2)

# #finding contours__________________________________________________________________________
		# ret,thresh = cv2.threshold(edges, 127, 255, 0)
		# contours,hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		# cv2.drawContours(dst, contours, -1, (0, 0, 255), -1)

# #defined points approach # 				
		spac = 30
		(rows,cols)=dst.shape
		# print(dst.shape)
		shared = str("False")
		counter = 0
		for i in range(int(rows)):
			for j in range(int(cols)):
				if i>250 and i < 300:
					if j>150 and j< 400:
						if dst[i,j]>100:
							if ((dst[i,j]<=2200)):
								counter = counter+1
								if counter> 500:
									print('distance = ',dst[i,j], 'row = ', i, 'column = ', j)
									shared = str("True")
									break
		with open("/home/max/shared.pkl","wb") as f:
			print(shared)
			pickle.dump(shared, f)
					
# #imshow outputs______________________________________________________________________   
# 		if(flag120[1:3]==[1, 1] and f12==1):
# 			#print flag, "FWD"
# 			cv2.putText(dst," frwd",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
# 		elif(flag120[2:4]==[1, 1] and f12==1):
# 			#print flag, "RIGHT"
# 			cv2.putText(dst," right",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
# 		elif(flag120[0:2]==[1, 1] and f12==1):
# 			#print flag, "LEFT"
# 			cv2.putText(dst," left",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
# 		elif(f12==1):
# 			#print flag, "BACK"
# 			cv2.putText(dst," back",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
# # 		cv2.line(dst,(480,0),(480,1080),(0),1)
# # 		cv2.line(dst,(960,0),(960,1080),(0),1)
# # 		cv2.line(dst,(1440,0),(1440,1080),(0),1)
		# cv2.imshow('Video', dst)

		listener.release(frames)
			
		key = cv2.waitKey(delay=1)
		if key == ord('q'):
			break
		
device.stop()
