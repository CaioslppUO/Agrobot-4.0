#!/usr/bin/env python

"""
@package encoder_reader
Read encoder values.
"""

from agrobot_services.log import Log
from agrobot_services.runtime_log import RuntimeLog
import traceback


# Log class
log: Log = Log("encoder_reader.py")
runtime_log: RuntimeLog = RuntimeLog("encoder_reader.py")
gpio_imported = False

try:
    import RPi.GPIO as GPIO
    gpio_imported: bool = True
except Exception as e:
    gpio_imported: bool = False
    log.warning("Could not import RPi.GPIO. {0}".format(traceback.format_exc()))
    runtime_log.warning("Could not import RPi.GPIO")

# Control variables
last_clk = -1
last_dt = -1
count = 0
clk_pin: int = None # Green pin
dt_pin: int = None # White pin

def set_pins(green: int, white: int):
    """
    Set the pinout for white and green encoder cables.
    """
    global clk_pin, dt_pin
    clk_pin = green
    dt_pin = white
    if(gpio_imported):
        # GPIO Configurations
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(clk_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(dt_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    runtime_log.info("Pin {0} set as Green and {1} as White".format(green, white))


def __read_encoder():
    """Read encoder values.
       Must have GPIO Library imported.
    """
    global clk_pin, dt_pin
    return GPIO.input(clk_pin), GPIO.input(dt_pin)

def __process_encoder_reading(clk, dt) -> int:
    """
    Process read values from encoder and update count value.
    Must have GPIO Library imported.
    """
    global last_clk, last_dt, count
    if(clk != last_clk or dt != last_dt):
            if(clk == dt):
                last_clk = clk
                last_dt = dt
                clk,dt = __read_encoder()
                # Wait until encoder change position
                while(last_clk == clk and last_dt == dt):
                    clk, dt = __read_encoder()
                # Calculate to which side it is turning
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
    return count

DETECT_READ_ERROR = False

def read() -> int:
    """
    Read, process and return the encoder value.
    """
    try:
        global clk_pin, dt_pin, DETECT_READ_ERROR
        clk, dt = __read_encoder()
        return __process_encoder_reading(clk, dt)
    except Exception as e:
        if(not DETECT_READ_ERROR):
            log.error(traceback.format_exc())
            runtime_log.error(traceback.format_exc())
            DETECT_READ_ERROR = True
        return 0