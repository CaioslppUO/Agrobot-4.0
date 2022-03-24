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

@socketio.on("power_motor")
def on_power_motor(data):
    emit("power_motor_changed", data, broadcast=True)

@socketio.on("auto_mode_activated")
def on_auto_mode_activated(data):
    emit("auto_mode_activated_changed", data, broadcast=True)

@socketio.on("module_activated")
def on_module_activated(data):
    emit("module_activated_changed", data, broadcast=True)

@socketio.on("type_module_control")
def on_type_module_control(data):
    emit("type_module_control_changed", data, broadcast=True)

@socketio.on("manual_wheel_adjustment_update")
def on_manual_wheel_adjustment_update(data):
    emit("manual_wheel_adjustment_update_changed", data, broadcast=True)

@socketio.on("auto_mode_params_update")
def on_auto_mode_params_update(data):
    emit("auto_mode_params_update_changed", data, broadcast=True)

@socketio.on("control_mode_update")
def on_auto_mode_params_update(data):
    emit("control_mode_changed", data, broadcast=True)

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
