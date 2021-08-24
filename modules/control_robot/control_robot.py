#!/usr/bin/env python

import os
import pathlib
import pwd
import threading
import threading
import rospy
from communication.server import wait_for_connection, connect
from agrobot.msg import Control
from agrobot.log import Log
from agrobot.param import Parameter


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
connections = []
address = []
t_motor = []
priorities: dict = {}


def get_rosparam_priorities() -> None:
    """
    Get priorities from rosparam.
    """
    global priorities
    priorities.update({"USB_PORT1": param.get_param("USB_PORT1")})
    priorities.update({"USB_PORT2": param.get_param("USB_PORT2")})
    priorities.update({"HTTP_PORT": int(param.get_param("HTTP_PORT"))})

# Function that communicates with socket server


def send(msg: Control):
    for con in connections:
        con.send(str(msg.speed).encode('utf-8'))

# Function that read and store clients connected with server


def storeConnections():
    global connections, address
    while True:
        conn, addr = wait_for_connection(s)
        connections.append(conn)
        address.append(addr)

# Start thread reference instance motor "Vesc"


def startThreadMotor(port: str):
    global t_motor
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()

# Function that start instance from motor


def startMotor(args):
    os.system("{0}controller.out {1}".format(communication_directory, args))


if __name__ == "__main__":
    global server
    try:
        param.wait_for_setup()
        get_rosparam_priorities()
        server = connect('localhost', priorities["HTTP_PORT"])
        t_connections = threading.Thread(target=storeConnections, args=())
        t_connections.start()
        startThreadMotor("-1 "+priorities["USB_PORT1"])
        startThreadMotor("1 "+priorities["USB_PORT2"])
        rospy.Subscriber('/control_robot', Control, send)
        rospy.spin()
    except Exception as e:
        for u_motor in t_motor:
            u_motor.close()
        for con in connections:
            con.close()
        log.error(str(e))
