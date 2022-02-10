
"""
@package tests
Execute all automated tests.
"""

from agrobot_services.param import Parameter

def test_setup():
    """
    Test the setup.py script.
    """
    test_ok = False
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
        test_ok = True
    except Exception as e:
        test_ok = False

    if(test_ok):
        print("setup.py: OK")
    else:
        print("setup.py: NO")

if __name__ == "__main__":
    try:
        test_setup()
    except Exception as e:
        print(str(e))