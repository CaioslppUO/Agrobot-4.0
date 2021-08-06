#!/usr/bin/env python

"""
@package priority_decider
Define which received command will be executed.
"""

import rospy
from std_msgs.msg import String
from agrobot.log import Log
from agrobot.param import Parameter

# priority_decider node
rospy.init_node("priority_decider", anonymous=True)

# Log class
log: Log = Log("priority_decider.py")

# Parameter class
param: Parameter = Parameter()

# Publication topic
pub = rospy.Publisher("priority_decider", String, queue_size=10)

# Control variables
current_priority: int = 0
remaining_commands: int = 0
current_command: int = 0
priorities: dict = {}


def get_rosparam_priorities() -> None:
    """
    Get priorities from rosparam.
    """
    global priorities
    priorities.update({"APP_PRIORITY": param.get_param("APP_PRIORITY")})
    priorities.update({"LIDAR_PRIORITY": param.get_param("LIDAR_PRIORITY")})
    priorities.update(
        {"GUARANTEED_COMMANDS": param.get_param("GUARANTEED_COMMANDS")})


def publish_selected_command(command: String) -> None:
    """
    Publish the selected command.
    """
    global current_command
    try:
        if(command != None):
            pub.publish(command)
            current_command = None
        else:
            log.warning("Could not publish command. Command is None.")
    except Exception as e:
        log.error(str(e))


def callback(data, priority: int) -> None:
    """
    Response to a listened command.

    Parameters:
    command -> Command read.
    priority -> Priority read.
    """
    global current_priority, remaining_commands, current_command
    if(priority >= current_priority or (remaining_commands == 0 and priority < current_priority)):
        current_priority = priority
        remaining_commands = priorities["GUARANTEED_COMMANDS"]
        current_command = str(data.data)
        publish_selected_command(current_command)
    else:
        remaining_commands -= 1


def listen(topic: str, priority: int) -> None:
    """
    Liste to a topic and call callback function.

    Parameters:
    topic -> Topic to be listened.
    priority -> Priority level of topic.
    """
    try:
        rospy.Subscriber(topic, String, callback, callback_args=priority)
    except Exception as e:
        log.error(str(e))


def add_listeners_and_listen() -> None:
    """
    Add all topics to be listened and listen to them.

    Parameters:
    """
    global current_command
    topics: dict = {
        "get_robot_commands": priorities["APP_PRIORITY"],
        "control_lidar": priorities["LIDAR_PRIORITY"]
    }
    for key in topics:
        listen(str(key), int(topics[key]))
    rospy.spin()


if __name__ == "__main__":
    try:
        param.wait_for_setup()
        get_rosparam_priorities()
        add_listeners_and_listen()
    except Exception as e:
        log.error(str(e))
