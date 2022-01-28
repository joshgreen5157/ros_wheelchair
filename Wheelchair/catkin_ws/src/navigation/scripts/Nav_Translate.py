#!/usr/bin/env python3

import time
import rospy
import serial
from nav_msgs.msg import Path

def setupComPort(comPort):
    serialPort = serial.Serial(port = comPort, baudrate = 9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    return serialPort

def writeCommand(comPort, str):
    comPort.write(str)

def commandThread(command):
    while (1):
   	 print(command)
   	 written = False
    return written

def mainThread(command, written):
    while(1):
        if command == "s" and written == False:
            print("I received a stop command")
            writeCommand(COM, "s")
            written = True
        elif command == "a" and written == False:
            print("I received a Automatic command")
            writeCommand(COM, bytes("a"))
            written = True
        elif command == "m" and written == False:
            print("I received a Manual command")
            writeCommand(COM, "m")
            written = True    
        elif command == "b" and written == False:
            print("I received a Back command")
            writeCommand(COM, "b")
            written = True
        elif command == "i" and written == False:
            print("I received an on command")
            writeCommand(COM, "i")
            written = True
        elif command == "f" and written == False:
            print("I received an forward command")
            writeCommand(COM, "f")
            written = True
        elif command == "r" and written == False:
            print("I received an right command")
            writeCommand(COM, "r")
            written = True
        elif command == "l" and written == False:
            print("I received an left command")
            writeCommand(COM, "l")
            written = True
        time.sleep(.001)
    return written
    


def callback(poses):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", poses)
    writeCommand(COM, 0)

def listener():
    rospy.init_node('listner',anonymous=True)
    rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, callback)
    rospy.spin()

if __name__ == '__main__':
    while(1):
        COM = setupComPort("/dev/ttyACM0")
        command = input("Enter a command: ")
        if command == 'STOP':
            writeCommand(COM, bytes('m', 'utf-8'))
            COM.close()
            quit()
        writeCommand(COM, bytes(command, 'utf-8'))
