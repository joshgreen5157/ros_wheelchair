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

# Translate the desired command and assign it the proper numeric value
def translateCommands(target):
    global COM
    lineA = float(target.linear.x)     ## linear.x range (0.0 to 0.5)
    lineB = float(target.angular.z)    ## angular.z range (-1.0 to 1.0)
    speed = 20*lineA ##0.1 = 20, 0.2 = 40, 0.3 = 60, 0.4 = 80, 0.5 = 100
    ## Assume positive theta = left turn, Negative theta is right turn
    slam_turn = 1 / lineB
    # if lineB < 0: #right
    # elif lineB > 0: #left turn
    # else: # lineB == 0 no turn

    ## kinect differential
    if obs_range == 0:
        speed = speed * 4
        kinect_turn_percentage = 1
    else
        speed = speed * obs_range
        #256 = do nothing so subtract 256 and then eval
        kinect_index = 256 - target_index
        kinect_turn_percentage = kinect_index / 256

    leftMotor = speed + (speed*slam_turn)
    rightMotor = speed - (speed*slam_turn)

    leftMotor = leftMotor + (speed*kinect_turn_percentage)
    rightMotor = rightMotor - (speed*kinect_turn_percentage)    
    

    print('x = ',target.linear.x,'a = ', lineA)
    print('y = ',target.angular.z,'b = ', lineB)
    writeCommand(COM, "%" + leftMotor + "&" + rightMotor)

# Format the desired command and send it over the open COM port
def writeCommand(comPort, strvar):
    comPort.write(str.encode(strvar + '*'))  

# Translate and send velocity commands received from rViz, flush serial line every 25 messages sent to prevent overloading
def navCommandsReceived(poses):
    global COM
    global freeze
    global serialCounter
    if freeze == "False":
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
    writeCommand(COM,'a')
    time.sleep(.5)
    writeCommand(COM,'a')


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
def checkCamera(pose):
    global COM
    if os.path.getsize("/home/max/shared.json") > 0: 
        fp = open("/home/max/shared.json", "r")
        kinect_dict = json.load("/home/max/shared.json")
        target_index = kinect_dict["target"]
        obs_range = kinect_dict["range"]
        if target_index == -1:
            print("No viable space")
            stopWheelchair()
        

# Send Stop command to wheelchair
def stopWheelchair():
    global COM
    writeCommand(COM, "%0&0")

# Looping listener for ROS Topics
def listener():
    global freeze
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('/pose2D', Pose2D, checkCamera)
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, newGoalReceived)
    rospy.Subscriber('/cmd_vel', Twist, navCommandsReceived)
    rospy.Subscriber('/move_base/status', GoalStatusArray, targetReached)
    rospy.spin()
  
# Launch ROS and rVIZ, start listener process
def main():
    p = mp.Process(target=ROSProcess)
    p.start()
    time.sleep(10)

    l = mp.Process(target=listener)
    l.start()

    # sp.run('mark3.py', shell = True, check = True, stdout = sp.PIPE, stderr = sp.STDOUT)
    time.sleep(5)
    print('Ready for target location')
    p.join()
    l.join()

if __name__ == '__main__':
    print('Start navigation script')
    main()
