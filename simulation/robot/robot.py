#!/usr/bin/env python

"""
@package robot
Simulate the main components of the robot for tests purpose.
"""
from agrobot.msg import Control
from agrobot_services.log import Log
import rospy, os, traceback

# Robot node
rospy.init_node("robot", anonymous=True)

log: Log = Log("robot.py")

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

class Wheel:
    def __init__(self) -> None:
        self.state = "still"

    def move_forward(self) -> None:
        self.state = "forward"

    def move_backward(self) -> None:
        self.state = "backward"

    def stop(self) -> None:
        self.state = "still"
    
class Encoder:
    def __init__(self) -> None:
        self.position = 90.0

    def increase_position(self) -> None:
        self.position += 0.01

    def decrease_position(self) -> None:
        self.position -= 0.01

class Motor:
    def __init__(self) -> None:
        self.encoder = Encoder()
        self.orientation = "still"

    def move_left(self) -> None:
        self.encoder.increase_position()
        self.orientation = "left"

    def move_right(self) -> None:
        self.encoder.decrease_position()
        self.orientation = "right"

    def stop(self) -> None:
        self.orientation = "still"

class Robot:
    def __init__(self) -> None:
        self.wheel_left = Wheel()
        self.wheel_right = Wheel()
        self.motor_left = Motor()
        self.motor_right = Motor()

    def move_axis(self, encoder_destine_value: int, motor: Motor):
        """
        Move the motor until reach the encoder_destine_value value.
        """
        encoder = motor.encoder.position
        dead_zone = 20
        if(encoder_destine_value >= 0 and encoder_destine_value <= 180):
            if( (encoder > 180 and encoder_destine_value < 90) or (encoder < 0 and encoder_destine_value > 90) or (encoder <= 180 and encoder >= 0) ):
                if(encoder_destine_value < 90 - dead_zone): # Go to left
                    motor.move_right()
                elif(encoder_destine_value > 90 + dead_zone): # Go to right
                    motor.move_left()
                else: # Stop
                    motor.stop()
            else:
                motor.stop()
        else:
            motor.stop()

    def move(self, wheel: Wheel, speed: float) -> None:
        """
        Move or stop the wheel. backward or forward.
        """
        if(speed > 0):
            wheel.move_forward()
        elif(speed < 0):
            wheel.move_backward()
        else:
            wheel.stop()

    def show_monitor(self) -> None:
        os.system("clear")
        print(color_text("----------> MONITOR <----------", bcolors.HEADER))
        print(color_text("Left Wheel", bcolors.OKGREEN), end="")
        print(color_text("             Right Wheel", bcolors.OKGREEN))
        print("position: {0:12} position: {1}".format(self.wheel_left.state, self.wheel_right.state))
        print("\n")
        print(color_text("Left Motor", bcolors.OKGREEN), end="")
        print(color_text("             Right Motor", bcolors.OKGREEN))
        print("encoder: {0:.2f} {1:7} encoder: {2:.2f}".format(self.motor_left.encoder.position, " ", self.motor_right.encoder.position))
        print("direction: {0:11} direction: {1}".format(self.motor_left.orientation, self.motor_right.orientation))

    def callback_control_robot(self, command: Control) -> None:
        """
        Response to a new command to movement the robot.
        """
        try:
            # Steer
            dst = 90 * (command.steer + 1)
            self.move_axis(dst, self.motor_left)
            self.move_axis(dst, self.motor_right)
            # Speed
            self.move(self.wheel_left, command.speed)
            self.move(self.wheel_right, command.speed)
            self.show_monitor()
        except Exception as e:
            log.error(traceback.format_exc())

    def listen_control_robot(self):
        """
        Listen to the control_robot topic and call the callback function
        """
        rospy.Subscriber("/control_robot", Control, self.callback_control_robot)
        rospy.spin()

if __name__ == "__main__":
    try:
        robot = Robot()
        robot.show_monitor()
        robot.listen_control_robot()
    except Exception as e:
        log.error(traceback.format_exc())

        
