#!/usr/bin/env python

"""
@package control_direction
Read encoder value and move the motors that control the robot direction.
"""

import rospy, sys
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot.msg import Control
from agrobot_services.runtime_log import RuntimeLog

# Log class
log: Log = Log("control_direction.py")
runtime_log: RuntimeLog = RuntimeLog("control_direction.py")

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(int(sys.argv[1]), GPIO.OUT)
    GPIO.setup(int(sys.argv[2]), GPIO.OUT)
    gpio_imported: bool = True
    runtime_log.info("Pin {0} set as Motor 1 and {2} as Motor 2".format(sys.argv[1], sys.argv[2]))
except Exception as e:
    gpio_imported: bool = False
    log.warning("Could not import RPi.GPIO.")
    runtime_log.warning("Could not import RPi.GPIO")

# Encoder node
rospy.init_node('control_direction', anonymous=True)

# Control variables.
encoder: int = 90
motor_1: int = int(sys.argv[1]) # Raspberry pin for motor 1
motor_2: int = int(sys.argv[2]) # Raspberry pin for motor 2

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
    dead_zone = 20
    if(goTo >= 0 and goTo <= 180):
        if( (encoder > 180 and goTo < 90) or (encoder < 0 and goTo > 90) or (encoder <= 180 and encoder >= 0) ):
            if(goTo < 90 - dead_zone): # Go to left
                turn_left()
            elif(goTo > 90 + dead_zone): # Go to right
                turn_right()
            else: # Stop
                stop()
        else:
            stop()
            runtime_log.info("Wheels stopped because encoder value reached 999 or -999")
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
    angle = 90
    dst = angle * (command.steer + 1)
    move(encoder, dst)
    

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
