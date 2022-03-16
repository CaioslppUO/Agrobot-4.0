#!/usr/bin/env python

import os
import pathlib
import pwd
from shutil import ExecError
import threading
import threading
from time import sleep
import rospy
from communication.server import wait_for_connection, connect
from agrobot.msg import Control
from agrobot_services.log import Log
from agrobot_services.param import Parameter
import traceback


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
def send(msg: Control) -> None:
    """
    Send command to socket client in C
    """
    for con in connections:
        con.send(str(msg.speed).encode('utf-8'))

# Function that read and store clients connected with server


def storeConnections():
    """
    Store connections socket 
    """
    global connections, address, server
    while True:
        conn, addr = wait_for_connection(server)
        connections.append(conn)
        address.append(addr)


def startThreadMotor(port: str):
    """
    Start threads that store program in C that controls Vesc
    """
    global t_motor
    sleep(5) #Pensar em uma logica para remover isso
             #Cliente esta tentando conectar no servidor
             #sem ele ter subido, causando erro na conexão.
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()
    print(port + " connected in server.")


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
        server = connect('localhost', priorities["HTTP_PORT"])
        t_connections = threading.Thread(target=storeConnections, args=())
        t_connections.start()
        startThreadMotor(priorities["USB_PORT1"])
        startThreadMotor(priorities["USB_PORT2"])
        rospy.Subscriber('/control_robot', Control, send)
        rospy.spin()
    except Exception as e:
        for u_motor in t_motor:
            u_motor.close()
        for con in connections:
            con.close()
        log.error(traceback.format_exc())
