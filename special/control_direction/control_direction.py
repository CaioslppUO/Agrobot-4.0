#!/usr/bin/env python

"""
@package control_direction
Read encoder value and move the motors that control the robot direction.
"""

import rospy, sys
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot.msg import Control, WheelAdjustment
from agrobot_services.runtime_log import RuntimeLog
import traceback

# Log class
log: Log = Log("control_direction.py")
runtime_log: RuntimeLog = RuntimeLog("control_direction.py")

# Variáveis de controle de publicação.
pub: rospy.Publisher = rospy.Publisher("/control_direction", String, queue_size=10)

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(int(sys.argv[1]), GPIO.OUT)
    GPIO.setup(int(sys.argv[2]), GPIO.OUT)
    gpio_imported: bool = True
    runtime_log.info("Pin {0} set as Motor 1 and {1} as Motor 2. - Wheel set {2} and {3}".format(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]))
except Exception as e:
    gpio_imported: bool = False
    log.warning("Could not import RPi.GPIO. {0}".format(traceback.format_exc()))
    runtime_log.warning("Could not import RPi.GPIO")

# Encoder node
rospy.init_node('control_direction', anonymous=True)

# Control variables.
encoder: int = 90
motor_1: int = int(sys.argv[1]) # Raspberry pin for motor 1
motor_2: int = int(sys.argv[2]) # Raspberry pin for motor 2
wheel_set_left = int(sys.argv[3]) # Number of the wheel_set for the left wheel
wheel_set_right = int(sys.argv[4]) # Number of the wheel_set for the right wheel

def stop(motor_1: bool = True, motor_2: bool = True) -> None:
    """
    Stop the motors.
    """
    if(gpio_imported):
        if(motor_1):
            GPIO.output(motor_1, GPIO.LOW)
        if(motor_2):
            GPIO.output(motor_2, GPIO.LOW)

def turn_right(motor_1: bool = True, motor_2: bool = True) -> None:
    """
    Turn the motors to the right indefinitely.
    """
    if(gpio_imported):
        if(motor_1):
            GPIO.output(motor_1, GPIO.LOW)
        if(motor_2):
            GPIO.output(motor_2, GPIO.HIGH)

def turn_left(motor_1: bool = True, motor_2: bool = True) -> None:
    """
    Turn the motors to the left indefinitely.
    """
    if(gpio_imported):
        if(motor_1):
            GPIO.output(motor_1, GPIO.HIGH)
        if(motor_2):
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
                turn_right()
                pub.publish("turn right")
            elif(goTo > 90 + dead_zone): # Go to right
                turn_left()
                pub.publish("turn left")
            else: # Stop
                stop()
                pub.publish("stop")
        else:
            stop()
            pub.publish("stop")
            runtime_log.info("Wheels stopped because encoder value reached > 180 or < 0")
    else:
        log.warning("Invalid steer value: {0}.".format(goTo))
    if(readValue < 0 or readValue > 300):
        log.warning("Invalid read value from encoder {0}.".format(readValue))

def move_one_wheel(readValue: int, goTo: int, wheel: int):
    """
    Move one motor until reach the goTo value.
    """
    global ecoder
    dead_zone = 20
    if(wheel == wheel_set_left or wheel == wheel_set_right):
        motor_1 = True
        motor_2 = True
        if(wheel == wheel_set_left):
            motor_1 = True
            motor_2 = False
        elif(wheel == wheel_set_right):
            motor_1 = False
            motor_2 = True
        if(goTo >= 0 and goTo <= 180):
            if( (encoder > 180 and goTo < 90) or (encoder < 0 and goTo > 90) or (encoder <= 180 and encoder >= 0) ):
                if(goTo < 90 - dead_zone): # Go to left
                    turn_right(motor_1, motor_2)
                    pub.publish("turn right")
                elif(goTo > 90 + dead_zone): # Go to right
                    turn_left(motor_1, motor_2)
                    pub.publish("turn left")
                else: # Stop
                    stop(motor_1, motor_2)
                    pub.publish("stop")
            else:
                stop()
                pub.publish("stop")
                runtime_log.info("Wheels stopped because encoder value reached > 180 or < 0")
        else:
            log.warning("Invalid steer value: {0}.".format(goTo))
        if(readValue < 0 or readValue > 300):
            log.warning("Invalid read value from encoder {0}.".format(readValue))

def callback(data) -> None:
    """
    Response to a selected command from priority decider.
    """
    global encoder
    try:
        encoder = int(data.data)
    except Exception as e:
        log.error(traceback.format_exc())
        pass

def move_callback(command: Control):
    """
    Response to the movement command.
    """
    global encoder
    angle = 90
    dst = angle * (command.steer + 1)
    move(encoder, dst)

def adjust_wheel_callback(command: WheelAdjustment):
    """
    Response to the wheel adjustment command.
    """
    global encoder
    angle = 90
    dst = angle * (command.direction + 1)
    move_one_wheel(encoder, dst, command.wheel)

def listen_encoder() -> None:
    """
    Listen to the priority decider topic and call the callback function
    """
    rospy.Subscriber("/encoder", String, callback)
    rospy.Subscriber("/control_robot", Control, move_callback)
    rospy.Subscriber("/wheel_adjustment", WheelAdjustment, adjust_wheel_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listen_encoder()
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("control_direction.py terminated")
