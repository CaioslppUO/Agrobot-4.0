#!/usr/bin/env python

"""
@package control_direction
Read encoder value and move the motors that control the robot direction.
"""

import rospy
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot.msg import Control
import time

# Log class
log: Log = Log("control_direction.py")

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(16, GPIO.OUT)
    GPIO.setup(19, GPIO.OUT)
    gpio_imported: bool = True
except Exception as e:
    gpio_imported: bool = False
    log.warning("Could not import RPi.GPIO.")

# Encoder node
rospy.init_node('control_direction', anonymous=True)

# Control variables.
encoder: int = 90
motor_1: int = 16 # Raspberry pin 16
motor_2: int = 19 # Raspberry pin 19

def stop() -> None:
    """
    Stop the motors.
    """
    if(gpio_imported):
        GPIO.output(motor_1, GPIO.LOW)
        GPIO.output(motor_2, GPIO.LOW)

def turn_right() -> None:
    """
    Turn the motors to the right indefinitely.
    """
    if(gpio_imported):
        GPIO.output(motor_1, GPIO.LOW)
        GPIO.output(motor_2, GPIO.HIGH)

def turn_left() -> None:
    """
    Turn the motors to the left indefinitely.
    """
    if(gpio_imported):
        GPIO.output(motor_1, GPIO.HIGH)
        GPIO.output(motor_2, GPIO.LOW)

def move(readValue: int, goTo: int):
    """
    Move the motor until reach the goTo value.
    """
    global ecoder
    dead_zone = 10
    if(goTo >= 0 and goTo <= 180):
        if(readValue == 999 or readValue == -999):
            stop()
            print("Stop")
        elif(goTo >= readValue-dead_zone and goTo <= readValue+dead_zone):
            stop()
            print("Stop")
        elif(goTo < readValue): # Go to left
            turn_left()
            print("Turn Left")
        elif(goTo > readValue): # Go to right
            turn_right()
            print("Turn Right")
        else: # Stop
            stop()
            print("Stop")
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
        pass

def move_callback(command: Control):
    """
    Response to the movement command.
    """
    global encoder
    command.steer = command.steer + 1
    angle = 90
    dst = angle * command.steer
    move(encoder,dst)
    

def listen_encoder() -> None:
    """
    Listen to the priority decider topic and call the callback function
    """
    rospy.Subscriber("/encoder", String, callback)
    rospy.Subscriber("/control_robot", Control, move_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listen_encoder()
    except Exception as e:
        log.error(str(e))
