from modeltruck_platooning.msg import drive_param
from mocap_source_2 import Mocap
from socket import *
import struct
import time
import sys
import numpy
import math


class Controller:

    def __init__(self, w = 0, v = 1):
        self.omega = w
        self.speed = v
        self.xM = x
        self.yM = y


    def translateInput(self, w):

        if(w > 0): #Turn left
            print("Turn Left")
            turnLeft(self.omega, self.speed)
        elif(w<0): #Turn right
            turnRight(self.omega, self.speed)
            print("Turn Right")
        else:
            rint("Keep on")


    def turnRight(self, w, v):

        r = 1   #svängradie 2..!
        theta = math.asin(w*r/v)
        microSec = 0

        if(theta > pi/6):
            microSec = 1200
        else:
            microSec = 1500 - 300*6/pi * theta

        print("New angle = ", theta)
        print("New input = ", microSec)

    def turnRight(self,w,v):

        r = 1  # svängradie 2..!
        theta = math.asin(w * r / v)
        microSec = 0;

        if (theta < -(pi/6)):
            microSec = 1800;
        else:
            microSec = 1500 + 300*6/pi*theta;

        print("New angle = ", theta)
        print("New input = ", microSec)

        return microSec




