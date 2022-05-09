#!/usr/bin/env python

"""
@package control_direction
Read control_robot topic and control the direction of the wheels.
"""

from sre_parse import WHITESPACE
import serial
import rospy
import sys
from agrobot.msg import Control
from std_msgs.msg import String
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.param import Parameter
from agrobot_services.log import Log
import traceback

# Constants
MOTOR_1 = "M1"  # Name of motor 1 in serial protocol
MOTOR_2 = "M2"  # Name of motor 2 in serial protocol
RANGE = 2048  # Values go from -2048 to +2048
WHEEL_SET_FRONT = 0
WHEEL_SET_BACK = 1
WHEEL_SET = int(sys.argv[1])  # Front or back wheel set.

# Control Modes
MODE_A = "A"
MODE_B = "B"
MODE_C = "C"
CONTROL_MODE = "C"

# Encoder
ENCODER_1 = 90  # 90 means centralized
ENCODER_2 = 90
ENCODER_3 = 90
ENCODER_4 = 90
TURN_LIMIT = 25
ENCODER_MAX = 180
ENCODER_MIN = 0

# Global variables
priorities: dict = {}

log: Log = Log("control_direction.py")
runtime_log: RuntimeLog = RuntimeLog("control_direction.py")

# Parameter class
param: Parameter = Parameter()

rospy.init_node("control_direction", anonymous=True)


def get_rosparam_priorities() -> None:
    """
    Get priorities from rosparam.
    """
    global priorities
    priorities.update(
        {"USB_PORT_SABERTOOTH": param.get_param("USB_PORT_SABERTOOTH")})


def _write(msg: str) -> None:
    """
    Write in the serial port.
    Automatically transform to bytes and insert \r\n at the end.
    """
    if(port):
        port.write(str('{}\r\n'.format(msg)).encode())
    else:
        runtime_log.warning(
            "Could not send control_direction to robot, but it would send {0}.".format(msg))


def turn_motor(motor: str, speed: int = 0) -> None:
    if(speed >= -RANGE and speed <= RANGE):
        _write("{0}: {1}".format(motor, int(speed)))


def set_control_mode(mode: str, left: int, right: int) -> tuple:
    if(mode == MODE_A):  # Only fron wheels turn
        if(WHEEL_SET == WHEEL_SET_FRONT):
            return (MOTOR_1, left, MOTOR_2, -left)
        else:
            return (MOTOR_1, 0, MOTOR_2, 0)
    elif(mode == MODE_B):  # Both turn
        return (MOTOR_1, left, MOTOR_2, -right)
    elif(mode == MODE_C):  # Oposite directions
        if(WHEEL_SET == WHEEL_SET_FRONT):
            return (MOTOR_1, left, MOTOR_2, -right)
        elif(WHEEL_SET == WHEEL_SET_BACK):
            return (MOTOR_1, -left, MOTOR_2, right)
        else:
            raise Exception("WHEEL_SET is not a valid value")
    else:
        raise Exception("Control mode not found")


def move_callback(command: Control) -> None:
    """
    Callback for moving the motors.
    """
    try:
        steer = command.steer * RANGE  # Speed at which the motor will turn
        # Turning both motors (Control mode should work here)
        m1, spd1, m2, spd2 = set_control_mode(CONTROL_MODE, steer, steer)
        # Encoder Verification
        if(WHEEL_SET == WHEEL_SET_FRONT):  # Encoder 1 and Encoder 2
            if(ENCODER_1 > ENCODER_MIN + TURN_LIMIT and ENCODER_1 < ENCODER_MAX - TURN_LIMIT):
                turn_motor(m1, spd1)
            else:
                runtime_log.log(
                    "Can't turn motor_1 because encoder_1 would be out of range")
            if(ENCODER_2 > ENCODER_MIN + TURN_LIMIT and ENCODER_2 < ENCODER_MAX - TURN_LIMIT):
                turn_motor(m2, spd2)
            else:
                runtime_log.log(
                    "Can't turn motor_2 because encoder_2 would be out of range")
        elif(WHEEL_SET == WHEEL_SET_BACK):  # Encoder 3 and Encoder 4
            if(ENCODER_3 > ENCODER_MIN + TURN_LIMIT and ENCODER_3 < ENCODER_MAX - TURN_LIMIT):
                turn_motor(m1, spd1)
            else:
                runtime_log.log(
                    "Can't turn motor_3 because encoder_3 would be out of range")
            if(ENCODER_4 > ENCODER_MIN + TURN_LIMIT and ENCODER_4 < ENCODER_MAX - TURN_LIMIT):
                turn_motor(m2, spd2)
            else:
                runtime_log.log(
                    "Can't turn motor_4 because encoder_4 would be out of range")
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error(traceback.format_exc())


def control_mode_callback(mode) -> None:
    """
    Listen to the topic and update the control mode.
    """
    global CONTROL_MODE
    CONTROL_MODE = str(mode.data)


def encoder_callback(value: String, encoder: int) -> None:
    """
    Update the value of the first encoder
    """
    global ENCODER_1, ENCODER_2, ENCODER_3, ENCODER_4
    if(encoder == 1):
        ENCODER_1 = int(value.data)
    elif(encoder == 2):
        ENCODER_2 = int(value.data)
    elif(encoder == 3):
        ENCODER_3 = int(value.data)
    elif(encoder == 4):
        ENCODER_4 = int(value.data)
    else:
        raise Exception("Unknown encoder number")


COULD_NOT_LISTEN_ENCODER = False


def listen_topics() -> None:
    """
    Listen to the topics needed to control the direction.
    """
    rospy.Subscriber("/change_mode", String, control_mode_callback)
    rospy.Subscriber("/control_robot", Control, move_callback)
    if(WHEEL_SET == WHEEL_SET_FRONT):
        rospy.Subscriber("/encoder_1", String, encoder_callback, (1))
        rospy.Subscriber("/encoder_2", String, encoder_callback, (2))
    elif(WHEEL_SET == WHEEL_SET_BACK):
        rospy.Subscriber("/encoder_3", String, encoder_callback, (3))
        rospy.Subscriber("/encoder_4", String, encoder_callback, (4))
    else:
        if(not COULD_NOT_LISTEN_ENCODER):
            log.error(
                "Could not listen to encoder because WHEEL_SET was not defined or was wrong")
            runtime_log.error(
                "Could not listen to encoder because WHEEL_SET was not defined or was wrong")
    rospy.spin()


if __name__ == "__main__":
    try:
        get_rosparam_priorities()
        try:
            # Port for serial communication
            port = serial.Serial(priorities["USB_PORT_SABERTOOTH"], 9600)
        except Exception as e:
            port = False
            log.error(traceback.format_exc())
        listen_topics()
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("control_direction.py terminated")
