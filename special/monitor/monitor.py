#!/usr/bin/env python

import os, rospy
from agrobot.msg import Control
from std_msgs.msg import String
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
import traceback
import time

from rich.live import Live
from rich.table import Table

# Monitor node
rospy.init_node("monitor", anonymous=True)

# Log
log: Log = Log("monitor.py")
runtime_log: RuntimeLog = RuntimeLog("monitor.py")

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def color_text(text: str, color: bcolors) -> str:
    """
    Paint the color of a text and returns it.
    """
    return color + text + bcolors.ENDC

class App_server:
    def __init__(self) -> None:
        self.speed: float = 0.0
        self.steer: float = 0.0
        self.limit: float = 0.0

    def show(self):
        print(color_text("          App Server", bcolors.OKGREEN))
        print("speed: {0}".format(self.speed))
        print("steer: {0}".format(self.steer))
        print("limit: {0}\n".format(self.limit))

class Control_robot:
    def __init__(self) -> None:
        self.speed: float = 0.0
        self.steer: float = 0.0
        self.limit: float = 0.0

    def show(self):
        print(color_text("          Control Robot", bcolors.OKGREEN))
        print("speed: {0}".format(self.speed))
        print("steer: {0}".format(self.steer))
        print("limit: {0}\n".format(self.limit))
        
class Encoder:
    def __init__(self) -> None:
        self.value: int = 0

    def show(self):
        print(color_text("          Encoder", bcolors.OKGREEN))
        print("value: {0}\n".format(self.value))


class Monitor:
    def __init__(self) -> None:
        self.app = App_server()
        self.control = Control_robot()
        self.encoder = Encoder()
        self.direction = "stop"

    def generate_table(self) -> Table:
        table = Table()
        table.add_column("source")
        table.add_column("speed")
        table.add_column("steer")
        table.add_column("limit")
        table.add_column("encoder")
        table.add_column("direction")

        table.add_row("App Server",
            "{0}".format(self.app.speed),
            "{0}".format(self.app.steer),
            "{0}".format(self.app.limit),
            "-"
            "-",
            "-"
        )

        table.add_row("Control Robot",
            "{0}".format(self.control.speed),
            "{0}".format(self.control.steer),
            "{0}".format(self.control.limit),
            "-"
            "-",
            "-"
        )

        table.add_row("Encoder",
            "-",
            "-",
            "-"
            "-",
            "{0}".format(self.encoder.value),
            "-"
        )

        table.add_row("Control Direction",
            "-",
            "-",
            "-"
            "-",
            "-",
            "{0}".format(self.direction)
        )
        
        return table

    def show(self):
        os.system("clear")
        self.app.show()
        self.control.show()
        self.encoder.show()
        print(color_text("          Control Direction", bcolors.OKGREEN))
        print("direction: {0}\n".format(self.direction))

    def callback_app(self, data: Control, live: Live):
        self.app.speed = data.speed
        self.app.steer = data.steer
        self.app.limit = data.limit
        live.update(self.generate_table())

    def callback_control(self, data: Control, live: Live):
        self.control.speed = data.speed
        self.control.steer = data.steer
        self.control.limit = data.limit
        live.update(self.generate_table())

    def callback_encoder(self, data: String, live: Live):
        self.encoder.value = int(data.data)
        live.update(self.generate_table())

    def callback_control_direction(self, data: String, live: Live):
        self.direction = str(data.data)
        live.update(self.generate_table())
    
    def listen(self) -> None:
        with Live(self.generate_table(), refresh_per_second=4) as live:
            rospy.Subscriber("/get_robot_commands", Control, self.callback_app, live)
            rospy.Subscriber("/control_robot", Control, self.callback_control, live)
            rospy.Subscriber("/encoder", String, self.callback_encoder, live)
            rospy.Subscriber("/control_direction", String, self.callback_control_direction, live)
            rospy.spin()

if __name__ == "__main__":
    try:
        m = Monitor()
        m.listen()  
    except Exception as e:
        print(traceback.format_exc())
        log.error(traceback.format_exc())
        runtime_log.error("control_direction.py terminated")
