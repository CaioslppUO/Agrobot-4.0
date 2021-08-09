import os
import threading
from server import *


def send():
    while True:
        msg = input('Digite algo para enviar: ')
        for con in connections:
            con.send(msg.encode('utf-8'))


def storeConnections():
    global connections, address
    while True:
        conn, addr = wait_for_connection(s)
        connections.append(conn)
        address.append(addr)


def startThreadMotor(port: str):
    t_motor = threading.Thread(target=startMotor, args=(str(port)))
    t_motor.start()


def startMotor(port: str):
    os.system("./controller.out " + port)


connections = []
address = []
s = connect('localhost', 3000)

try:
    startThreadMotor("/dev/ttyACM0")
    # startThreadMotor("/dev/ttyACM1")
    t_connections = threading.Thread(target=storeConnections, args=())
    t_connections.start()

    send()
except Exception as e:
    for con in connections:
        con.close()
    print(str(e))
