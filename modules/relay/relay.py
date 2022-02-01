#!/usr/bin/env python3

"""
@package relay
Controla a comunicação com os relés que controlam módulos extras, através do GPIO.
"""

import sys,time,rospy
from std_msgs.msg import Bool
from agrobot_services.runtime_log import RuntimeLog
from agrobot_services.log import Log
import traceback

# Variáveis de controle de importação.
gpio_imported: bool = False

log: Log = Log("relay.py")
runtime_log: RuntimeLog = RuntimeLog("relay.py")

try:
    import RPi.GPIO as GPIO
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    gpio_imported = True
except Exception as e:
    runtime_log.error(str(e))
    print(str(e))


# Nó do relé.
rospy.init_node("relay")

## Envia o sinal para o relé que liga ou desliga o módulo extra..
def power_control_callback(data: Bool) -> None:
    pinout: int = int(sys.argv[1])        
    try:
        if(data.data):
            if(gpio_imported):
                runtime_log.info("Relay On with gpio")
                GPIO.setup(pinout, GPIO.OUT)
                GPIO.output(pinout, GPIO.HIGH)
                time.sleep(0.2)
            else:
                runtime_log.info("Relay On without gpio")
        else:
            if(gpio_imported):
                runtime_log.info("Relay Off with gpio")
                GPIO.setup(pinout, GPIO.OUT)
                GPIO.output(pinout, GPIO.LOW)
                time.sleep(0.2)
            else:
                runtime_log.info("Relay Off without gpio")
    except Exception as e:
        runtime_log.error(str(e))
        print(str(e))
        


## Escuta comandos recebidos que devem ser enviados para o relé.
def listen_relay() -> None:
    rospy.Subscriber("module_activated", Bool, power_control_callback)
    rospy.spin()

if __name__ == "__main__":
    try:
        listen_relay()
    except Exception as e:
        log.error(traceback.format_exc())
        runtime_log.error("relay.py terminate");