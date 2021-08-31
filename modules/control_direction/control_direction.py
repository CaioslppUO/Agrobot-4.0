#!/usr/bin/env python

"""
@package control_direction
Read encoder value and move the motors that control the robot direction.
"""

import rospy
from std_msgs.msg import String
from agrobot.log import Log
from agrobot.msg import Control

# Log class
log: Log = Log("control_direction.py")

# Encoder node
rospy.init_node('control_direction', anonymous=True)

# Control variables.
encoder: int = 90

def move(readValue: int, goTo: int):
    global ecoder
    if(goTo > 0 and goTo < 180):
        if(goTo > readValue): # Go to right
            pass
        else: # Go to left
            pass
    else:
        log.warning("Invalid steer value: {0}".format(goTo))
    if(readValue < 0 or readValue > 300):
        log.warning("Invalid read value from encoder {0}".format(readValue))

def callback(data) -> None:
    """
    Response to a selected command from priority decider.
    """
    global encoder
    try:
        encoder = int(data.data)
    except Exception as e:
        log.error(str(e))

def move_callback(command: Control):
    global encoder
    

def listen_encoder() -> None:
    """
    Listen to the priority decider topic and call the callback function
    """
    rospy.Subscriber("/encoder", String, callback)
    rospy.Subscriber("/control_robot", Control, move_callback)
    rospy.spin()