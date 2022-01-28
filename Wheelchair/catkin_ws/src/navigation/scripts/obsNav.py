import numpy as np
import cv2
import scipy.misc
import signal
import pyfreenect2
from numpy import testing
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
    np.clip(depth, 0, 2**10 - 1, depth)
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
  
frameListener = pyfreenect2.SyncMultiFrameListener(pyfreenect2.Frame.COLOR, pyfreenect2.Frame.IR,pyfreenect2.Frame.DEPTH)

registration = Registration(device.getIrCameraParams(),device.getColorCameraParams())

undistorted = Frame(512, 424, 4)
registered = Frame(512, 424, 4)

listener = SyncMultiFrameListener(FrameType.Color | FrameType.Ir | FrameType.Depth)

device.setColorFrameListener(listener)
device.setIrAndDepthFrameListener(listener)
device.start()

bigdepth = Frame(1920, 1082, 4)
color_depth_map = np.zeros((424, 512),  np.int32).ravel() 

cv2.namedWindow('Video')
cv2.moveWindow('Video',5,5)
# cv2.namedWindow('Navig',cv2.WINDOW_AUTOSIZE)
# cv2.resizeWindow('Navig',400,100)
# cv2.moveWindow('Navig',700,5)
kernel = np.ones((5, 5), np.uint8)

# print('Press \'b\' in window to stop')
# cv2.createTrackbar('val1', 'Video', 37, 1000, nothing)
# cv2.createTrackbar('val2', 'Video', 43, 1000, nothing)
# cv2.createTrackbar('bin', 'Video',20,50,nothing)
# cv2.createTrackbar('erode', 'Video',4,10,nothing)#after plenty of testing
# imn=cv2.imread('blank.bmp')

while 1:
	if listener.hasNewFrame():
		frames = listener.waitForNewFrame()
		color = frames["color"]
		ir = frames["ir"]
		depth = frames["depth"]
		registration.apply(color, depth, undistorted, registered,bigdepth=bigdepth,color_depth_map=color_depth_map)

		flag120=[1, 1, 1, 1]
		flag140=[1, 1, 1, 1]
		f14=0
		f12=0
		f10=0
		f8=0

# #get kinect input__________________________________________________________________________
		dst = pretty_depth(cv2.resize(depth.asarray(),(int(1920), int(1080))))

# #rectangular border (improved edge detection + closed contours)___________________________ 
		cv2.rectangle(dst,(0,0),(1920,1080),(40,100,0),2)
	   
# #image binning (for distinct edges)________________________________________________________
		# binn=cv2.getTrackbarPos('bin', 'Video') 
		# e=cv2.getTrackbarPos('erode', 'Video') 
		# dst = (dst/binn)*binn
		# dst = cv2.erode(dst, kernel, iterations=e)
		# dst = np.uint8(dst)
		
# #Video detection___________________________________________________________________________
		v1 = cv2.getTrackbarPos('val1', 'Video')
		v2 = cv2.getTrackbarPos('val2', 'Video')
		edges = cv2.Canny(dst, v1, v2)
		# cv2.imshow('Canny', edges)

# #finding contours__________________________________________________________________________
		ret, thresh = cv2.threshold(edges, 127, 255, 0)
		contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		cv2.drawContours(dst, contours, -1, (0, 0, 255), -1)

# #finding contour center of mass (moments)___________________________________________________
		cx=0
		cy=0
		try:
			for i in range(len(contours)):
					M = cv2.moments(contours[i])
		
					cx = int(M['m10']/M['m00'])
					cy = int(M['m01']/M['m00'])
					cv2.circle(dst, (cx, cy), 6, (0, 255, 0), 3)
			cx = int(cx/len(contours))
			cy = int(cy/len(contours))
		except:
			pass

# #boundingRect approach_______________________________________________________________________
		cv2.createTrackbar('epsilon', 'Video', 1, 100, nothing)#for approxPolyDP
		ep=cv2.getTrackbarPos('epsilon', 'Video') 

# #defined points approach (to check: runtime)________________________________________________
		cv2.createTrackbar('spacing', 'Video', 30, 100, nothing)#for approxPolyDP
		spac = cv2.getTrackbarPos('spacing', 'Video') 
		(rows,cols)=dst.shape

# 	#print cols
		for i in range(int(rows/spac)):
			for j in range(int(cols/spac)):
				cv2.circle(dst, (spac*j,spac*i), 1, (0, 255, 0), 1)
				if (dst[spac*i,spac*j]==80):
					print("0")
					print(dst)
					# f8=1
					# cv2.putText(dst,"0",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),2)
					# cv2.putText(dst,"Collision Alert!",(30,30),cv2.FONT_HERSHEY_TRIPLEX,1,(2),1)
					# imn=cv2.imread("Collision Alert.bmp")
					# cv2.imshow('Navig',imn)
				if (dst[spac*i,spac*j]==100):
					print("1")
					print(dst)					# f10=1
					# cv2.putText(dst,"1",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),2)
					# cv2.putText(dst,"Very Close proximity. Reverse",(30,60),cv2.FONT_HERSHEY_TRIPLEX,1,(2),1)
					# if(f8==0):
					# 	imn=cv2.imread("VCP Reverse.bmp")
					# 	cv2.imshow('Navig',imn)
				if (dst[spac*i,spac*j]==120):
					print("2")
					print(dst)
					# f12=1
					# cv2.putText(dst, "2", (spac*j, spac*i), cv2.FONT_HERSHEY_PLAIN, 1, (0, 200, 20), 2)
					# flag120 = RegionCheck(spac*j, flag120)
					# if(f8==0 and f10==0):
					# 	imgshow(flag120,120,imn,'Navig')
				if (dst[spac*i,spac*j]==140):
					print("3")
					print(dst)
					# f14=1
					# cv2.putText(dst,"3",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),1)
					# flag140 = RegionCheck(spac*j, flag140)
					# if(f8==0 and f10==0 and f12==0):
					# 	imgshow(flag140,140,imn,'Navig')
				if (dst[spac*i,spac*j]==160):
					cv2.putText(dst,"4",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),1)
				if (dst[spac*i,spac*j]==180):
					cv2.putText(dst,"5",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),1)
				if (dst[spac*i,spac*j]==200):
					cv2.putText(dst,"6",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),1)
				if (dst[spac*i,spac*j]==220):
					cv2.putText(dst,"7",(spac*j,spac*i),cv2.FONT_HERSHEY_PLAIN,1,(0,200,20),1)
	
#imshow outputs______________________________________________________________________   
		if(flag120[1:3]==[1, 1] and f12==1):
			#print flag, "FWD"
			cv2.putText(dst," frwd",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
		elif(flag120[2:4]==[1, 1] and f12==1):
			#print flag, "RIGHT"
			cv2.putText(dst," right",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
		elif(flag120[0:2]==[1, 1] and f12==1):
			#print flag, "LEFT"
			cv2.putText(dst," left",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
		elif(f12==1):
			#print flag, "BACK"
			cv2.putText(dst," back",(325,90),cv2.FONT_HERSHEY_DUPLEX,1,(2),1)
		cv2.line(dst,(130,0),(130,480),(0),1)
		cv2.line(dst,(320,0),(320,480),(0),1)
		cv2.line(dst,(510,0),(510,480),(0),1)
		cv2.imshow('Video', dst)

		listener.release(frames)
			
		key = cv2.waitKey(delay=1)
		if key == ord('q'):
			break