import os
import threading

def startThreadMotor(port: str):
    global t_motor
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()


def startMotor(args):
    os.system("./controller.out " + args)


t_motor = []

try:
    startThreadMotor("/dev/ttyACM0")
    startThreadMotor("/dev/ttyACM1")
    while True:
        pass
except Exception as e:
    for u_motor in t_motor:
        u_motor.close()
    print(str(e))
