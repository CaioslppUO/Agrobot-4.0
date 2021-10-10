#!/usr/bin/env python3

"""
@package setup
Load to rosparam variables and constants used by the application.
"""

from agrobot_services.log import Log
from agrobot_services.param import Parameter

# Log class
log: Log = Log("setup.py")

# Parameter class
param: Parameter = Parameter()

# Parameters
parameters = {
    # Priorities
    "APP_PRIORITY": "1000",
    "LIDAR_PRIORITY": "999",
    "GUARANTEED_COMMANDS": "50",
    "GUARANTEED_COMMANDS": "50",
    # Control
    "USB_PORT1": "/dev/ttyACM0",
    "USB_PORT2": "/dev/ttyACM1",
    "HTTP_PORT": "3509",
    # Setup
    "SETUP_DONE": "True"
}


def setup_parameters() -> None:
    """
    Upload all parameters to ros.
    """
    for key in parameters:
        param.set_param(key, parameters[key])


if __name__ == "__main__":
    try:
        setup_parameters()
    except Exception as e:
        log.error(str(e))
