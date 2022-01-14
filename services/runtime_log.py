#!/usr/bin/env python

"""
@package runtime_log
Show log lines in the console.
"""

from datetime import datetime

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class RuntimeLog:
    def __init__(self, file_name: str) -> None:
        self.file_name = file_name

    def __get_current_time(self) -> str:
        """
        Return time in format H:M:S.
        """
        return datetime.today().strftime("%H:%M:%S")
    
    def __color_text(self, text: str, color: bcolors) -> str:
        """
        Paint the color of a text and returns it.
        """
        return color + text + bcolors.ENDC

    def info(self, message) -> None:
        """
        Print a info log in terminal.
        """
        print("({0})[{1}]<{2}> - {3}".format(self.__color_text("INFO", bcolors.OKCYAN), self.file_name, self.__get_current_time(), message))

    def warning(self, message) -> None:
        """
        Print a warning log in terminal.
        """
        print("({0})[{1}]<{2}> - {3}".format(self.__color_text("WARNING", bcolors.WARNING), self.file_name, self.__get_current_time(), message))
    
    def error(self, message) -> None:
        """
        Print a error log in terminal.
        """
        print("({0})[{1}]<{2}> - {3}".format(self.__color_text("ERROR", bcolors.FAIL), self.file_name, self.__get_current_time(), message))