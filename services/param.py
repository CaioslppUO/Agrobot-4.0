#!/usr/bin/env python

"""
@package param
Used to get and set rosparam parameters.
"""

import rosparam, traceback
from agrobot_services.log import Log

# Log class
log: Log = Log("param.py", ignore_start=True)

"""
Manage rosparam parameters.
"""


class Parameter:
    def __init__(self):
        pass

    def set_param(self, name: str, value: str) -> None:
        """
        Set a new rosparam parameter.

        Parameters:
        name -> Name of the parameter.
        value -> Value of the parameter.
        """
        try:
            rosparam.set_param(name, value)
            log.info(
                "New rosparam parameter set: {0} = {1}".format(name, value))
        except Exception as e:
            log.error(traceback.format_exc())

    def get_param(self, name: str) -> str:
        """
        Get a parameter from rosparam.

        Parameters:
        name -> Name of the parameter.
        """
        try:
            return rosparam.get_param(name)
        except Exception as e:
            raise e

    def wait_for_setup(self) -> bool:
        """
        Wait for setup to be executed. If timeout is reached, return false.
        """
        setup_done: bool = False
        timeout: int = 0
        while(not setup_done and timeout <= 100):
            timeout += 1
            try:
                a = rosparam.get_param("SETUP_DONE")
                setup_done = True
            except Exception as e:
                pass
        return timeout < 100
