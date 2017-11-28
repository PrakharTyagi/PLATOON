
import numpy
import math


class Translator:
    def __init__(self, w = 0, v = 1):
        self.omega = float(w)
        self.speed = float(v)
        self.speedMicro = 1500;
        self.microSec = 0
        self.angle = 0

    def translateInput(self, w):
        if (w >= 0):
            #print("Turn Left")
            leftTurn = True;

        elif (w < 0):
            leftTurn = False
            #print("Turn Right")

        self.turn(w, self.speed, leftTurn)

    def turn(self, w, v,leftTurn):

        l = 0.25
        x = l/2
        r = v/w
        if r**2 - x**2 > 0:
            den = math.sqrt(r**2-x**2)
            a = math.atan(l/den)
        else:
            a = math.pi/6
        microSec = 1500

        if(leftTurn == False):
            a= a*(-1)

        if(a> math.pi/6):
            microSec = 1800
            a = math.pi/6

        elif((a < -math.pi/6)):
            microSec = 1200
            a = -math.pi/6
        else:
            microSec = 1500 + 300*6/math.pi*a

        self.microSec = microSec
        self.angle = a*180/math.pi

    def getMicroSec(self):
        return self.microSec

    def getAngle(self):
        return self.angle

    def getSpeed(self):
        return self.speedMicro

    def getGear(self):
        return 60
