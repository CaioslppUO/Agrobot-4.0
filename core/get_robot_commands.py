#!/usr/bin/env python3

"""
@package get_robot_commands
Get control data from app server and send it to ROS server.
"""

import rospy,os,pathlib,json,requests,traceback
from agrobot_services.log import Log
from agrobot.msg import Control, WheelAdjustment
from shutil import which
from agrobot_services.runtime_log import RuntimeLog

# Directory variables
current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"

# get_robot_commands node
rospy.init_node('get_robot_commands', anonymous=True)

# Log class
log: Log = Log("get_robot_commands.py")
runtime_log: RuntimeLog = RuntimeLog("get_robot_commands.py")

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
        cm.speed = float(command["speed"]) * float(command["limit"])
        cm.steer = float(command["steer"])
        cm.limit = float(command["limit"])
        return cm
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not setup Control command")

def setup_wheel_command(command) -> WheelAdjustment:
    """
    Separate and assemble the command to control the robot.

    Parameters:
    command -> Json content with speed and steer values.
    """
    try:
        cm: WheelAdjustment = WheelAdjustment()
        cm.wheel = float(command["wheel"])
        cm.direction = float(command["direction"])
        return cm
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not setup WheelAdjustment command")

def publish_command(command: Control) -> None:
    """
    Publish the command to get_robot_commands topic.
    """
    global last_command
    try:
        if(last_command != command):
            pub = rospy.Publisher("/get_robot_commands", Control, queue_size=10)
            pub.publish(command)
            last_command = command
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not publish new command to /get_robot_commands")

def publish_wheel_adjustment(data: WheelAdjustment) -> None:
    try:
        pub = rospy.Publisher("/wheel_adjustment", WheelAdjustment, queue_size=10)
        pub.publish(data)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not publish new command to /wheel_adjustment")
    
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
            log.error(traceback.format_exc())
            runtime_log.error("Could not get IPV4")
    else:
        log.error("Could not find ifconfig tool. Please install package net-tools. {0}".format(traceback.format_exc()))
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

def get_robot_commands_wheel(ip: str):
    """
    Get the command to control the robot from the app server.
    Publish the command to ROS.
    """
    try:
        data = json.loads(requests.get(ip).content.decode('utf-8'))
        publish_wheel_adjustment(setup_wheel_command(data))
    except Exception as e:
        pass

if __name__ == '__main__':
    try:
        ip: str = str("http://" + get_ipv4() +":3000/control")
        ip_wheel_adjustment: str = str("http://" + get_ipv4() +":3000/manual_wheel_adjustment")
        if(ip != ""):
            while not rospy.is_shutdown():
                get_robot_commands(ip)
                get_robot_commands_wheel(ip_wheel_adjustment)
        else:
            log.error("Could not get ipv4.".format())
    except rospy.ROSInterruptException:
        log.warning("Roscore was interrupted.")