#!/usr/bin/env python

"""
@package priority_decider
Define which received command will be executed.
"""

import rospy,rosparam
from agrobot.log import Log
from agrobot.param import Parameter

# priority_decider node
rospy.init_node("priority_decider", anonymous=True)

# Log class
log: Log = Log("priority_decider.py")

# Parameter class
param: Parameter = Parameter()

if __name__ == "__main__":
    try:
        param.wait_for_setup()
    except Exception as e:
        log.error(str(e))