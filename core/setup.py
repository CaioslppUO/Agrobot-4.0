#!/usr/bin/env python3

"""
@package setup
Load to rosparam variables and constants used by the application.
"""

from agrobot_services.log import Log
from agrobot_services.param import Parameter
import traceback
from agrobot_services.runtime_log import RuntimeLog

# Log class
log: Log = Log("setup.py", "w")
runtime_log: RuntimeLog = RuntimeLog("setup.py")

# Parameter class
param: Parameter = Parameter()

# Parameters
parameters = {
    # Priorities
    "APP_PRIORITY": "1000",
    "LIDAR_PRIORITY": "999",
    "GUARANTEED_COMMANDS": "50",
    # Control
    "USB_PORT_SABERTOOTH": "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.1.3:1.1",
    # esquerda baixo
    "USB_PORT_VESC1": "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.2:1.0",  # direita cima
    "USB_PORT_VESC2": "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.3:1.0",  # direita baixo
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
        log.error(traceback.format_exc())
        runtime_log.error("setup.py terminated")
