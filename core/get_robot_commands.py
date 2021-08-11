#!/usr/bin/env python3

"""
@package get_robot_commands
Get control data from app server and send it to ROS server.
"""

import rospy,os,pathlib,json,requests
from agrobot.log import Log
from agrobot.msg import Control
from shutil import which

# Directory variables
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

# get_robot_commands node
rospy.init_node('get_robot_commands', anonymous=True)

# Log class
log: Log = Log("get_robot_commands.py")

# Command sending control 
last_command = None

def setup_command(command) -> Control:
    """
    Separate and assemble the command to control the robot.

    Parameters:
    command -> Json content with speed and steer values.
    """
    try:
        cm: Control = Control()
        cm.speed = float(command["speed"])
        cm.steer = float(command["steer"])
        return cm
    except Exception as e:
        log.error(str(e))

def publish_command(command: Control) -> None:
    """
    Publish the command to get_robot_commands topic.

    Parameters:
    command -> String with speed and steer values.
    """
    global last_command
    try:
        if(last_command != command):
            pub = rospy.Publisher("/get_robot_commands", Control, queue_size=10)
            pub.publish(command)
            last_command = command
    except Exception as e:
        log.error(str(e))
    
def get_ipv4() -> str:
    """
    Return ipv4.
    """
    ip: str = ""
    if(which("ifconfig") is not None):
        try:
            os.system("ifconfig | grep 'inet *.*.*.*' > " + current_directory + "ipv4.tmp")
            with open(current_directory+"ipv4.tmp","r") as file:
                line = file.readlines()
                file.close()
                line = line[0]
                line = line.split("inet ")[1].split(" ")
                ip = line[0]
            if(os.path.exists(current_directory+"ipv4.tmp")):
                os.system("rm " + current_directory+"ipv4.tmp")
        except Exception as e:
            log.error(str(e))
    else:
        log.error("Could not find ifconfig tool. Please install package net-tools.")
    return str(ip)

def get_robot_commands(ip: str):
    """
    Get the command to control the robot from the app server.
    Publish the command to ROS.
    """
    try:
        data = json.loads(requests.get(ip).content.decode('utf-8'))
        publish_command(setup_command(data))
    except Exception as e:
        pass

if __name__ == '__main__':
    try:
        ip: str = str("http://" + get_ipv4() +":3000/control")
        if(ip != ""):
            while not rospy.is_shutdown():
                get_robot_commands(ip)
        else:
            log.error("Could not get ipv4.")
    except rospy.ROSInterruptException:
        log.warning("Roscore was interrupted.")
