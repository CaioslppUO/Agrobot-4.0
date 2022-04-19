#!/usr/bin/env python

import os
import pathlib
import pwd
from shutil import ExecError
import threading
import threading
import rospy
from communication.server import wait_for_connection, connect
from agrobot.msg import Control
from agrobot_services.log import Log
from agrobot_services.param import Parameter
import traceback
import paho.mqtt.client as paho


# Parameter class
param: Parameter = Parameter()


log: Log = Log("control_robot.py")

user: str = str(pwd.getpwuid(os.getuid())[0])
home: str = "/home/{0}/".format(user)
current_directory: str = str(pathlib.Path("__file__").parent.absolute())
communication_directory: str = "{0}Agrobot/catkin_ws/src/agrobot/src/modules/control_robot/communication/".format(
    home)

# Control robot node
rospy.init_node("control_robot", anonymous=True)


# Global variables
t_motor = []
priorities: dict = {}
client = paho.Client()


def get_rosparam_priorities() -> None:
    """
    Get priorities from rosparam.
    """
    global priorities
    priorities.update({"USB_PORT1": param.get_param("USB_PORT_VESC1")})
    priorities.update({"USB_PORT2": param.get_param("USB_PORT_VESC2")})

# Function that communicates with socket server


def send(msg: Control) -> None:
    """
    Send command to socket client in C
    """

    client.publish("command/vesc", str(msg.speed))


def startThreadMotor(port: str):
    """
    Start threads that store program in C that controls Vesc
    """
    global t_motor
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()
    print(port + " started instance.")


def startMotor(args):
    """
    Start program in C that controls Vesc
    """
    os.system("{0}controller.out {1}".format(communication_directory, args))


if __name__ == "__main__":
    global server
    try:
        param.wait_for_setup()
        get_rosparam_priorities()
        if client.connect("localhost", 1883, 60) != 0:
            raise Exception("Could not connect to MQTT Broker!")
        startThreadMotor(priorities["USB_PORT1"])
        startThreadMotor(priorities["USB_PORT2"])
        rospy.Subscriber('/control_robot', Control, send)
        rospy.spin()
    except Exception as e:
        for u_motor in t_motor:
            u_motor.close()
        log.error(traceback.format_exc())
