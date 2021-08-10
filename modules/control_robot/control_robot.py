#!/usr/bin/env python

import os,pathlib,pwd
import threading
import rospy
from communication.server import wait_for_connection, connect
from agrobot.msg import Control

user: str = str(pwd.getpwuid(os.getuid())[0])
home: str = "/home/{0}/".format(user)
current_directory: str = str(pathlib.Path("__file__").parent.absolute())
communication_directory: str = "{0}Agrobot/catkin_ws/src/agrobot/src/modules/control_robot/communication/".format(home)

# Control robot node
rospy.init_node("control_robot", anonymous=True)

connections = []
address = []
s = connect('localhost', 3011)

def send(msg: Control):
    for con in connections:
        con.send(str(msg.speed).encode('utf-8'))

def storeConnections():
    global connections, address
    while True:
        conn, addr = wait_for_connection(s)
        connections.append(conn)
        address.append(addr)


def startThreadMotor(port: str):
    global t_motor
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()


def startMotor(args):
    os.system("{0}controller.out {1}".format(communication_directory,args))


t_motor = []
if __name__ == "__main__":
    try:
        t_connections = threading.Thread(target=storeConnections, args=())
        t_connections.start()
        startThreadMotor("/dev/ttyACM0")
        #startThreadMotor("/dev/ttyACM1")
        rospy.Subscriber('/control_robot', Control, send)
        rospy.spin()
    except Exception as e:
        for u_motor in t_motor:
            u_motor.close()
        for con in connections:
            con.close()
        print(str(e))
