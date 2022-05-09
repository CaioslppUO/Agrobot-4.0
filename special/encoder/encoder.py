#!/usr/bin/env python

"""
@package encoder
Process encoder values publishing them into ROS.
"""

import rospy, sys
from std_msgs.msg import String
from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
import traceback
from encoder_reader import read, set_pins
import multiprocessing as mp
from threading import Thread

# Log class
log: Log = Log("encoder.py")
runtime_log: RuntimeLog = RuntimeLog("encoder.py")

# Encoder node
rospy.init_node('encoder', anonymous=True)

# Variáveis de controle de publicação.
TOPIC_TO_PUB: str = sys.argv[3] # Topic to publish the encoder value
pub: rospy.Publisher = rospy.Publisher("/{}".format(TOPIC_TO_PUB), String, queue_size=10)

# Control variables
last_published_value = 89

def publish_encoder(value: str) -> None:
    """Publish processed encoder value."""
    global last_published_value
    if(int(value) != int(last_published_value)):
        pub.publish(value)
        print(value)
        last_published_value = value

def convertToDegrees(value: int) -> str:
    """Convert encoder value to degrees - 0 -> 180"""
    if(value > 300):
        return -1
    elif(value < -300):
        return 181
    OldRange = (300 - -300)  
    NewRange = (180 - 0)  
    NewValue = (((value - -300) * NewRange) / OldRange) + 0
    return str(int(NewValue))

def main():
    try:
        p = psutil.Process()
        p.cpu_affinity([2])
        set_pins(int(sys.argv[1]), int(sys.argv[2]))
        while not rospy.is_shutdown():
            encoder = read() # Read data from encoder
            publish_encoder(convertToDegrees(encoder)) # Publish encoder data to ROS
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error(traceback.format_exc())

if __name__ == "__main__":
    try:
        Thread(target=main).start()
    except Exception as e:
        print(traceback.format_exc())

