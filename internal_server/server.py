import socket
import threading

buffer: list = []
attemptive: int = 0
timeout: int = 10000

def connect(server_ip: str, port: int):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((server_ip, port))
    s.listen(1)
    return s

def wait_for_connection(connection):
    conn, addr = connection.accept()
    print("connection from: " + str(addr))
    return conn, addr

def transmission_bug_detected() -> bool:
    if(int(str(buffer[0])) != len(buffer)): return 1
    if(buffer[-1] != '$'): return 1
    return 0

def emit():
    global buffer

    string = ""
    for element in buffer:
        string += element
    buffer = []

    commands = string.split(";")
    for cm in commands:
        buffer.append(cm)

    if(transmission_bug_detected()): raise Exception("Transmission error: Size <{}> received but message length is <{}>".format(int(buffer[0]),len(buffer)))
    
    #Emit
    print(buffer)
    buffer = []

def process(msg: str):
    global buffer,attemptive
    attemptive += 1
    rec = ""
    for aux in msg.split("\x00"):
        rec += aux
    buffer.append(rec)

    if(attemptive >= timeout):
        attemptive = 0
        raise Exception("Process timeout: Can't find <$> at end of message.")

    if(buffer[-1][-1] == '$'): # End of command
        emit()
        
def receive():
    while True:
        data = conn.recv(10000)
        if str(data.decode("utf-8"))!="":
            process(str(data.decode("utf-8")))

def send(msg: str):
    msg+="\0"
    conn.send(msg.encode('utf-8'))

def aux():
    while True:
        au = input('Digite algo para enviar: ')
        send(au)

s = connect('localhost',3000)
conn, addr = wait_for_connection(s)

try:
    t_recieve = threading.Thread(target=receive, args=())
    t_recieve.start()
    aux()
except Exception as e:
    conn.close()
    print(str(e))

