#!/usr/bin/env python3

import os
from flask import Flask
from flask_socketio import SocketIO, emit

# Flask Server
app = Flask(__name__)
socketio = SocketIO(app)

@socketio.on("connect")
def on_connection():
    print("New Connection")

@socketio.on("control_update")
def on_control_update(data):
    emit("control_update_changed", data, broadcast=True)

@socketio.on("manual_wheel_adjustment_update")
def on_control_update(data):
    emit("manual_wheel_adjustment_update_changed", data, broadcast=True)

def start_server():
    socketio.run(app, host="0.0.0.0", port=3000)

def close_port():
    try:
        os.system("fuser -n tcp 3000 > temp.txt")
        f = open("temp.txt", "r")
        porta = f.readline()
        f.close()
        if(porta != ""):
            os.system("kill -9 " + str(porta))
            os.remove("temp.txt")
    except:
        print("Ninguém esta usando a porta 3000")

if __name__ == "__main__":
    try:
        close_port()
        start_server()
    except Exception as e:
        print(str(e))
