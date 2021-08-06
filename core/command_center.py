#!/usr/bin/env python

"""
@package command_center
Split and send each command to it's destination.
"""

import rospy
from std_msgs.msg import String
from agrobot_services.log import Log

# Log class
log: Log = Log("command_center.py")

# command_center node
rospy.init_node("command_center", anonymous=True)

# Control variables
pub_control: rospy.Publisher = rospy.Publisher("/control_robot", String, queue_size=10)

def send_command_to_robot(command: String) -> None:
    """
    Send a command to move the robot. 

    Parameters:
    command -> String with the command to move the robot.
    """
    pub_control.publish(command)

def callback(data: String) -> None:
    """
    Response to a selected command from priority decider.
    """
    try:
        send_command_to_robot(data.data)
    except Exception as e:
        log.error(str(e))

def listen_priority_decider() -> None:
    """
    Listen to the priority decider topic and call the callback function
    """
    rospy.Subscriber("/priority_decider", String, callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listen_priority_decider()
    except Exception as e:
        log.error(str(e))