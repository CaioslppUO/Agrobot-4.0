#!/usr/bin/env python

"""
@package control_direction
Read control_robot topic and control the direction of the wheels.
"""

import serial, rospy
from agrobot.msg import Control
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
import traceback

## Constants
MOTOR_1 = "M1" # Name of motor 1 in serial protocol
MOTOR_2 = "M2" # Name of motor 2 in serial protocol
RANGE = 2048 # Values go from -2048 to +2048

# Control Modes
MODE_EQUAL = "EQUAL"
MODE_INDIVIDUAL = "INDIVIDUAL"

port = serial.Serial('/dev/ttyACM0', 9600) # Port for serial communication

def _write(msg: str) -> None:
    """
    Write in the serial port.
    Automatically transform to bytes and insert \r\n at the end.
    """
    port.write(b'{0}\r\n'.format(msg))

def turn_motor(motor: str, speed: int = 0) -> None:
    if(speed >= -RANGE and speed <= RANGE):
        _write("{0}: {1}".format(motor, speed))

def turn_left(motor: str) -> None:
    turn_motor(motor, -RANGE)

def turn_right(motor: str) -> None:
    turn_motor(motor, RANGE)

def set_control_mode(mode: str, left: int, right: int) -> None:
    if(mode == MODE_EQUAL):
        return (MOTOR_1, left, MOTOR_2, left)
    elif(mode == MODE_INDIVIDUAL):
        return (MOTOR_1, left, MOTOR_2, right)
    else:
        raise("Control mode not found")

def move_callback(command: Control) -> None:
    """
    Callback for moving the motors.
    """
    try:
        steer = command.steer * RANGE # Speed at which the motor will turn

        # Turning both motors (Control mode should work here)
        m1, s1, m2, s2 = set_control_mode(0, steer, steer)
        turn_motor(m1, s1)
        turn_motor(m2, s2)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error(traceback.format_exc())

def listen_topics() -> None:
    """
    Listen to the topics needed to control the direction.
    """
    rospy.Subscriber("/control_robot", Control, move_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listen_topics()
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("control_direction.py terminated")