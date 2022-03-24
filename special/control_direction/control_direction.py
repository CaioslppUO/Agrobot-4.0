#!/usr/bin/env python

"""
@package control_direction
Read control_robot topic and control the direction of the wheels.
"""

from sre_parse import WHITESPACE
import serial, rospy, sys
from agrobot.msg import Control
from std_msgs.msg import String
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
import traceback

## Constants
MOTOR_1 = "M1" # Name of motor 1 in serial protocol
MOTOR_2 = "M2" # Name of motor 2 in serial protocol
RANGE = 2048 # Values go from -2048 to +2048
WHEEL_SET_FRONT = 0
WHEEL_SET_BACK = 1
WHEEL_SET = int(sys.argv[1]) # Front or back wheel set.

# Control Modes
MODE_A = "A"
MODE_B = "B"
MODE_C = "C"
CONTROL_MODE = "C"

log: Log = Log("control_direction.py")
runtime_log: RuntimeLog = RuntimeLog("control_direction.py")

rospy.init_node("control_direction", anonymous=True)

try:
    port = serial.Serial('/dev/ttyACM0', 9600) # Port for serial communication
except Exception as e:
        port = False
        log.error(traceback.format_exc())
        #runtime_log.error(traceback.format_exc())

def _write(msg: str) -> None:
    """
    Write in the serial port.
    Automatically transform to bytes and insert \r\n at the end.
    """
    if(port):
        port.write(str('{}\r\n'.format(msg)).encode())

def turn_motor(motor: str, speed: int = 0) -> None:
    if(speed >= -RANGE and speed <= RANGE):
        _write("{0}: {1}".format(motor, int(speed)))

def set_control_mode(mode: str, left: int, right: int) -> None:
    if(mode == MODE_A): # Only fron wheels turn
        if(WHEEL_SET == WHEEL_SET_FRONT):
            return (MOTOR_1, left, MOTOR_2, left)
    elif(mode == MODE_B): # Both turn
        return (MOTOR_1, left, MOTOR_2, right)
    elif(mode == MODE_C): # Oposite directions
        if(WHEEL_SET == WHEEL_SET_FRONT):
            return (MOTOR_1, left, MOTOR_2, right)
        elif(WHEEL_SET == WHEEL_SET_BACK):
            return (MOTOR_1, -left, MOTOR_2, -right)
    else:
        raise Exception("Control mode not found")

def move_callback(command: Control) -> None:
    """
    Callback for moving the motors.
    """
    try:
        steer = command.steer * RANGE # Speed at which the motor will turn
        # Turning both motors (Control mode should work here)
        m1, s1, m2, s2 = set_control_mode(CONTROL_MODE, steer, steer)
        turn_motor(m1, s1)
        turn_motor(m2, s2)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error(traceback.format_exc())

def control_mode_callback(mode) -> None:
    """
    Listen to the topic and update the control mode.
    """
    global CONTROL_MODE
    CONTROL_MODE = str(mode.data)

def listen_topics() -> None:
    """
    Listen to the topics needed to control the direction.
    """
    rospy.Subscriber("/change_mode", String, control_mode_callback)
    rospy.Subscriber("/control_robot", Control, move_callback)

    rospy.spin()

if __name__ == "__main__":
    try:
        listen_topics()
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("control_direction.py terminated")
