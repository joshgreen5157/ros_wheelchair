#!/usr/bin/env python3

import os
import time
import rospy
import serial
import pickle
import json
import actionlib
import subprocess as sp
import multiprocessing as mp
from std_msgs.msg import String
from geometry_msgs.msg import  PoseStamped,Twist, PoseWithCovarianceStamped, Pose2D
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction


# Launch ROS process
def ROSProcess():
    sp.run('roslaunch navigation hector_map.launch', shell = True, check = True, stdout = sp.PIPE, stderr = sp.STDOUT)

# Establish serial communication with external device
def setupComPort(comPort):
    serialPort = serial.Serial(port = comPort, baudrate = 9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    return serialPort

# Create necessary global variables 
COM = setupComPort("/dev/ttyACM0")
serialCounter = 0
cancelBool = False
obs_range = 0
target_index = 256
lastLeftMotor = 0
lastRightMotor = 0

# Clear the map on rViz (NOT CURRENTLY IN USE, IF USED ALLOW TIME FOR MAP TO REPOPULATE)
def clearMap():
    print("Clearing MAP")
    clear_publisher = rospy.Publisher("syscommand", String, queue_size=5)
    msg = "reset"
    clear_publisher.publish(msg)

# send a new target goal to rViz (NOT CURRENTLY IN USE)
def setGoal(msg):
    goal_publisher = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=5)
    goal = PoseStamped()
    if msg.pose != goal.pose:
        writeCommand(COM, 'a')

    goal.header.seq = 1
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
    goal.pose = msg.pose
    goal_publisher.publish(goal)    
    time.sleep(2)  

def limitMotor(motor):
    max = 50
    min = -max
    if motor > max:
        motor = max
    elif motor < min:
        motor = min
    else:
        motor = motor
    return motor

# Translate the desired command and assign it the proper numeric value
def translateCommands(target):
    global COM
    global lastLeftMotor
    global lastRightMotor
    global target_index
    global obs_range
    x = float(target.linear.x)     ## linear.x range (0.0 to 0.5)
    theta = float(target.angular.z)    ## angular.z range (-1.0 to 1.0)
    if x > 0:
        speed = 50
    if x < 0:
        speed = -25
       
    #ROS MOTOR VALUES
    if theta < 0:
        leftMotor = speed
        rightMotor = speed - theta*17
    elif theta > 0:
        leftMotor = speed - theta*17
        rightMotor = speed
    else:
        leftMotor = speed
        rightMotor = speed

    ## kinect differential
    if obs_range == 0 or target_index == 256:
        kinect_turn_percentage = 0
    else:
        speed = speed * obs_range/2 #do nothing unless object is within 2 meters
        #256 = do nothing so subtract 256 and then eval
        kinect_index = 256 - target_index
        kinect_turn_percentage = kinect_index / 750

    leftMotor = leftMotor + (speed*kinect_turn_percentage)
    rightMotor = rightMotor - (speed*kinect_turn_percentage)  

    leftMotor = limitMotor(leftMotor)
    rightMotor = limitMotor(rightMotor)  
    
    if abs(leftMotor - lastLeftMotor) > 50:
        leftMotor = lastLeftMotor + leftMotor / 2
    
    if abs(rightMotor - lastRightMotor) > 50:
        rightMotor = lastRightMotor + rightMotor / 2

    leftMotor = limitMotor(leftMotor)
    rightMotor = limitMotor(rightMotor)

    lastLeftMotor = leftMotor
    lastRightMotor = rightMotor

    writeCommand(COM, "%" + str(int(rightMotor)) + "&" + str(int(-leftMotor)))

# Format the desired command and send it over the open COM port
def writeCommand(comPort, strvar):
    comPort.write(str.encode(strvar + '*'))  

# Translate and send velocity commands received from rViz, flush serial line every 25 messages sent to prevent overloading
def navCommandsReceived(poses):
    global COM
    global serialCounter
    print("Nav Command Received")
    if target_index != -1:
        translateCommands(poses)
        if serialCounter == 25:
            COM.flushInput()
            COM.flushOutput()
            serialCounter = 0
        serialCounter = serialCounter+1

# When new target goal is reached, send an 'a' command to put the wheelchair in autonomous mode
def newGoalReceived(target):
    global COM
    global cancelBool 
    cancelBool = True

# When Target location is reach send a DONE command and clear the goal from Rviz
def targetReached(status):
    global COM
    global cancelBool
    if status.status_list != []:
        if status.status_list[0].status == 3 and cancelBool == True:
            print('Target reached')
            writeCommand(COM, 'DONE')
            move_base = actionlib.SimpleActionClient('/servicebot/move_base', MoveBaseAction)
            move_base.cancel_all_goals()
            cancelBool = False

# Check the camera output for Wheelchair Freeze command
cameraCount = 0
def checkCamera(pose):
    global COM
    global target_index
    global obs_range
    global cameraCount
    if cameraCount < 4:
        cameraCount += 1
        return
    print("Check Camera")
    with open("/home/max/Documents/share.json", "r") as fp:
        kinect_dict = json.load(fp)
        target_index = kinect_dict["target"]
        obs_range = kinect_dict["range"]
    if target_index == -1:
        print("No viable space")
        stopWheelchair()
    cameraCount = 0
        

# Send Stop command to wheelchair
def stopWheelchair():
    global COM
    global lastLeftMotor
    global lastRightMotor
    lastLeftMotor = 0
    lastRightMotor = 0
    writeCommand(COM, "%0&0")

# Looping listener for ROS Topics
def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('/pose2D', Pose2D, checkCamera)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, newGoalReceived)
    rospy.Subscriber('/cmd_vel', Twist, navCommandsReceived)
    rospy.Subscriber('/move_base/status', GoalStatusArray, targetReached)
    rospy.spin()
  
# Launch ROS and rVIZ, start listener process
def main():
    # p = mp.Process(target=ROSProcess)
    # p.start()
    time.sleep(10)

    l = mp.Process(target=listener)
    l.start()

    # sp.run('mark3.py', shell = True, check = True, stdout = sp.PIPE, stderr = sp.STDOUT)
    time.sleep(5)
    print('Ready for target location')
    # p.join()
    l.join()

if __name__ == '__main__':
    print('Start navigation script')
    main()
