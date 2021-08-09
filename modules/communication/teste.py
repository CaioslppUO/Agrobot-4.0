import os
import threading
import rospy
from server import wait_for_connection, connect


connections = []
address = []
s = connect('localhost', 3000)


def send(msg):
    for con in connections:
        con.send(msg.encode('utf-8'))


def storeConnections():
    global connections, address
    while True:
        conn, addr = wait_for_connection(s)
        connections.append(conn)
        address.append(addr)


def startThreadMotor(port: str):
    global t_motor
    t_motor.append(threading.Thread(target=startMotor, args=[port]))
    t_motor[len(t_motor)-1].start()


def startMotor(args):
    os.system("./controller.out " + args)


t_motor = []
if __name__ == "__main__":
    try:
        t_connections = threading.Thread(target=storeConnections, args=())
        t_connections.start()
        startThreadMotor("/dev/ttyACM0")
        startThreadMotor("/dev/ttyACM1")
        rospy.Subscriber('/control_robot', Control, send)
        rospy.spin()
    except Exception as e:
        for u_motor in t_motor:
            u_motor.close()
        for con in connections:
            con.close()
        print(str(e))
