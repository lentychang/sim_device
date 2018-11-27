#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function
import socket
import time
import pdb
import sys

def printErr(*args, **kwargs):
    print(*args, file=sys.stderr, **kwargs)


def printInfo(*args, **kwargs):
    print(*args, file=sys.stdout, **kwargs)


class Wsg50Gcl_Socket():
    def __init__(self, target_host="172.31.1.160", target_port=1000):
        ''' Create a socket
        '''
        # AF_INET means here we use standard IPv4 address or hostname
        # SOCK_STREAM means
        self.socketClient = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__msg = ""
        self.__gripperStatsTable = ["IDLE", "GRASPING", "NO PART", "HOLDING", "RELEASING", "POSITIONING", "ERROR"]
        self.__gripperStats = "No yet connected"
        self.__target_host = target_host
        self.__target_port = 1000

    def connect(self):
        '''Establish a connection
        '''
        self.socketClient.connect((self.__target_host, self.__target_port))
        # Enable debug info for system status
        self.__sendCommand("VERBOSE=1")
        self.ack_fast_stop()
        # if(self.socketClient.connect((target_host, target_port)) > 0):
        #     printInfo("Socket connection succeed!")
        #     self.__sendCommand("VERSION?", "WSG 50 Firmware Version:")
        # else:
        #     printErr("Socket connection failed")

    def __sendCommand(self, cmd, wait=1, *args):
        self.socketClient.send(cmd + "\n")
        time.sleep(wait)
        self.__msg = self.socketClient.recv(4096)
        printInfo("Msg received:")
        printInfo(self.__msg, *args)

    def homing(self):
        self.__sendCommand("HOME()")

    def ack_fast_stop(self):
        self.__sendCommand("FSACK()")

    def currentPos(self):
        self.__sendCommand("POS?")

    def currentForce(self):
        self.__sendCommand("FORCE?")

    def currentSpeed(self):
        self.__sendCommand("SPEED?")

    def gripStats(self):
        self.__sendCommand("GRIPSTATS")
        self.__gripperStats = self.__gripperStatsTable[int(self.__msg)]
        
    def disconnect(self):
        self.__sendCommand("BYE()")
        printInfo("Gripper disconnected")

    def grip(self, force=None, width=None, speed=None, *args):
        n_arg = 0
        if force is not None:
            if force > 0 and force < 80:
                n_arg += 1
            else:
                printErr("Please assign the force within [0,80] N")
                n_arg = -1

        if n_arg == 1 and width is not None:
            if width >= 0 and width <= 110:
                n_arg += 1
            else:
                printErr("Please assign the witdth within [0,110] mm")
                n_arg = -1

        if n_arg == 2 and speed is not None:
            # Speed limit must be checked
            if speed >= 0 and speed <= 200:
                n_arg += 1
            else:
                printErr("Please assign the speed within [0,200] mm/s")
                n_arg = -1

        if n_arg == 0:
            self.__sendCommand("GRIP()")
        elif n_arg == 1:
            self.__sendCommand("GRIP({0})".format(force))
        elif n_arg == 2:
            self.__sendCommand("GRIP({0},{1})".format(force, width))
        elif n_arg == 3:
            self.__sendCommand("GRIP({0},{1},{2})".format(force, width, speed))
        elif n_arg == -1:
            printErr("keyword arg Error")
        else:
            printErr("Unexpected Error")

    def release(self, pullback_dist=None, speed=None):
        n_arg = 0
        if pullback_dist is not None:
            if pullback_dist >= 0 and pullback_dist <= 55:
                n_arg += 1
            else:
                printErr("Please assign the pullback_dist within [0,55] mm")
                n_arg = -1

        if n_arg == 1 and speed is not None:
            if speed >= 0 and speed <= 200:
                n_arg += 1
            else:
                printErr("Please assign the speed within [0,110] mm/s")
                n_arg = -1

        if n_arg == 0:
            self.__sendCommand("RELEASE()")
        elif n_arg == 1:
            self.__sendCommand("RELEASE({0})".format(pullback_dist))
        elif n_arg == 2:
            self.__sendCommand("RELEASE({0},{1})".format(pullback_dist, speed))
        elif n_arg == -1:
            printErr("keyword arg Error")
        else:
            printErr("Unexpected Error")
    def move(self, width=55, speed=None):
        if width > 0 and width <= 110:
            if speed is None:
                self.__sendCommand("MOVE({0})".format(width))
            else:
                if speed > 0 and speed < 200:
                    self.__sendCommand("MOVE({0},{1})".format(width,speed))
                else:
                    printErr("speed out of range, it should between 0-200")
        else:
            printErr("position out of range, it should be between 0-55 mm")


if __name__ == "__main__":
    wsg_socket = Wsg50Gcl_Socket()
    wsg_socket.connect()

    pdb.set_trace()

    wsg_socket.homing()
    wsg_socket.grip()

