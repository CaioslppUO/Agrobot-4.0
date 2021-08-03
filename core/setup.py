#!/usr/bin/env python3

"""
@package setup
Carrega para o Rosparam as variáveis e constantes utilizadas pela aplicação.
"""

import rosparam
from agrobot.log import Log
from agrobot.param import Parameter

# Log class
log: Log = Log("setup.py")

# Parameter class
param: Parameter = Parameter()

# Parameters
parameters = {
    ## Priorities
    "APP_PRIORITY": "1000",
    "LIDAR_PRIORITY": "999",
    "GUARANTEED_COMMANDS": "50",
    ## Setup
    "SETUP_DONE": "True"
}

def setup_parameters() -> None:
    """
    Upload all parameters to ros.
    """
    for key in parameters:
        param.set_param(key,parameters[key])

if __name__ == "__main__":
    try:
        setup_parameters()
    except Exception as e:
        log.error(str(e))