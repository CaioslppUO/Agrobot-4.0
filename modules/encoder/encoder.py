#!/usr/bin/env python

"""
@package encoder
Read and process encoder values, publishing them into ROS.
"""

import rospy
from std_msgs.msg import String
from agrobot_services.log import Log

# Log class
log: Log = Log()

try:
    import RPi.GPIO as GPIO
    gpio_imported: bool = True
except Exception as e:
    gpio_imported: bool = False
    log.warning("Could not import RPi.GPIO.")

# Encoder pins
clk_pin = 7 # Green
dt_pin = 13 # White

# GPIO Configurations
if(gpio_imported):
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(clk_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(dt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Encoder node
rospy.init_node('control_robot', anonymous=True)


# Variáveis de controle de publicação.
pub: rospy.Publisher = rospy.Publisher("/encoder", String, queue_size=10)

# Control variables
last_clk = -1
last_dt = -1
count = 0

def read_encoder():
    """Read encoder values and process them.
       Must have GPIO Library imported.
    """
    return GPIO.input(clk_pin),GPIO.input(dt_pin)

def process_encoder_reading(clk,dt) -> int:
    """
    Process read values from encoder and update count value.
    Must have GPIO Library imported.
    """
    global last_clk,last_dt,count
    if(clk != last_clk or dt != last_dt):
            if(clk == dt):
                last_clk = clk
                last_dt = dt
                clk,dt = read_encoder()
                while(last_clk == clk and last_dt == dt):
                    clk,dt = read_encoder()
                if(last_clk == 1):
                    if(clk == 0 and dt == 1):
                        count += 1
                    elif(clk == 1 and dt == 0):
                        count -= 1
                elif(last_clk == 0):
                    if(clk == 1 and dt == 0):
                        count += 1
                    elif(clk == 0 and dt == 1):
                        count -= 1
            else:
                pass
            last_clk = clk
            last_dt = dt

def publish_encoder(value: str) -> None:
    """Publish processed encoder value."""
    pub.publish(value)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            if(gpio_imported):
                clk,dt = read_encoder()
                process_encoder_reading(clk,dt)
                publish_encoder(str(count))
    except Exception as e:
        log.error(str(e))