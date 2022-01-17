#!/usr/bin/env python

"""
@package command_center
Split and send each command to it's destination.
"""

import rospy
from agrobot.msg import Control
from agrobot_services.log import Log
import traceback
from agrobot_services.runtime_log import RuntimeLog

# Log class
log: Log = Log("command_center.py")
runtime_log: RuntimeLog = RuntimeLog("command_center.py")

# command_center node
rospy.init_node("command_center", anonymous=True)

# Control variables
pub_control: rospy.Publisher = rospy.Publisher("/control_robot", Control, queue_size=10)

def send_command_to_robot(command: Control) -> None:
    """
    Send a command to move the robot. 

    Parameters:
    command -> String with the command to move the robot.
    """
    pub_control.publish(command)

def callback(command: Control) -> None:
    """
    Response to a selected command from priority decider.
    """
    try:
        send_command_to_robot(command)
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("Could not send command to robot")

def listen_priority_decider() -> None:
    """
    Listen to the priority decider topic and call the callback function
    """
    rospy.Subscriber("/priority_decider", Control, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listen_priority_decider()
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("command_center.py terminated")