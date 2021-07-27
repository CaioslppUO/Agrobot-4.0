#!/usr/bin/env python3

import os
import pathlib

current_directory: str = str(pathlib.Path(__file__).parent.absolute()) + "/"


def start_server():
    os.system("cd " + current_directory+" && yarn start& ")


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
