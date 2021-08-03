#!/usr/bin/env python

import os,pathlib
from datetime import datetime

"""
Manage logs file.
"""
class Log:
    def __init__(self, src_file: str) -> None:
        """
        Constructor.

        Parameters:
        src_file -> File that is using the log class. 
        """
        self.file: str = src_file
        self.log_dir: str = str(pathlib.Path.home()) + "/Agrobot/catkin_ws/src/agrobot/logs/"
        self.log_file: str = self.log_dir+"logs.txt"

        self.__create_log_file()

    def __create_log_file(self) -> None:
        """
        Create the log folder and file.
        If log file already exists nothing happen.
        """
        try:
            if(not os.path.exists(self.log_dir)):
                os.mkdir(self.log_dir)
            with open(self.log_file,"a") as file:
                file.write("")
                file.close()
        except Exception as e:
            print(e)

    def __get_date(self) -> str:
        """
        Return the current date formated as day/month/year hours/minutes/seconds.
        """
        return datetime.today().strftime("%d/%m/%Y %H:%M:%S")

    def __write_log(self, type: str, message: str) -> None:
        """
        Write a log message to the log file.

        Parameters:
        type -> Type of log.
        message -> Text to be written.
        """
        try:
            date: str = self.__get_date()
            with open(self.log_file,"a") as file:
                file.write("({0})[{1}]<{2}> -> {3}\n".format(date,type,self.file,message))
                file.close()
        except Exception as e:
            print(e)


    def error(self, message: str) -> None:
        """
        Write an error log message.

        Parameters:
        message -> Text to be written.
        """
        self.__write_log("ERROR",message)

    def warning(self, message: str) -> None:
        """
        Write an warning message.

        Parameters:
        message -> Text to be written.
        """
        self.__write_log("WARNING",message)

    def info(self, message: str) -> None:
        """
        Write an information message.

        Parameters:
        message -> Text to be written.
        """
        self.__write_log("INFO",message)