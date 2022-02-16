
"""
@package tests
Execute all automated tests.
"""

from agrobot_services.param import Parameter
from agrobot.msg import Control
import socketio, time, rospy

# Socketio
sio = socketio.Client()

# Setup
setup_test_ok = True

# Server
number_of_server_tests = 0
number_of_successful_server_tests = 0

# Get Robot Commands
number_of_robot_commands_tests = 0
number_of_successful_robot_commands_tests = 0

# 

def test_setup():
    """
    Test the setup.py script.
    """
    global setup_test_ok
    setup_test_ok = False
    parameter = Parameter()
    parameter.wait_for_setup()
    parameters = [
        "APP_PRIORITY",
        "LIDAR_PRIORITY",
        "GUARANTEED_COMMANDS",
        "USB_PORT1",
        "USB_PORT2",
        "HTTP_PORT",
        "SETUP_DONE"
    ]
    try:
        for p in parameters:
            parameter.get_param(p)
        setup_test_ok = True
    except Exception as e:
        setup_test_ok = False

    if(setup_test_ok):
        print("setup.py: OK")
    else:
        print("setup.py: NO")

def test_server():
    """
    Test the server script.
    """
    # Socketio connection handler
    global number_of_successful_server_tests, number_of_server_tests
    number_of_server_tests = 7
    number_of_successful_server_tests = 0
    global sio

    @sio.on("control_update_changed")
    def control_update(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1

    @sio.on("power_motor_changed")
    def power_motor(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1

    @sio.on("auto_mode_activated_changed")
    def auto_mode(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1

    @sio.on("module_activated_changed")
    def module(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1
    
    @sio.on("type_module_control_changed")
    def type_control(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1

    @sio.on("manual_wheel_adjustment_update_changed")
    def wheel_adjustment(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1

    @sio.on("auto_mode_params_update_changed")
    def auto_mode_params(data):
        global number_of_successful_server_tests
        number_of_successful_server_tests += 1
        sio.disconnect()

    @sio.on("disconnect")
    def disconnect():
        global number_of_successful_server_tests, number_of_server_tests
        if(number_of_server_tests == number_of_successful_server_tests):
            print("start_server.py: OK")
        else:
            print("start_server.py: NO")

    connected = False
    timeout = 20 # 20 seconds to execute a timeout
    iteration = 0
    while(not connected):
        if(iteration == timeout):
            print("start_server.py: NO -> Timeout")
            break
        try:
            sio.connect('http://localhost:3000')
            connected = True
        except Exception as e:
            pass
        time.sleep(1)
        iteration += 1

    sio.emit("control_update", data={"steer": 0, "speed": 0, "limit": 1})  
    sio.emit("power_motor", data={})  
    sio.emit("auto_mode_activated", data={})  
    sio.emit("module_activated", data=False)  
    sio.emit("type_module_control", data={})  
    sio.emit("manual_wheel_adjustment_update", data={"wheel": 0, "direction": 0})  
    sio.emit("auto_mode_params_update", data={})  

    # Wait some time before disconnect the socketio client.
    iteration = 0
    while(iteration < timeout):
        iteration += 1
        time.sleep(1)
    sio.disconnect()

if __name__ == "__main__":
    try:
        test_setup()
        test_server()
    except Exception as e:
        print(str(e))