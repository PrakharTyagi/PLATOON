
import numpy
import math


class Controller:
    def __init__(self, w = 0, v = 1):
        self.omega = float(w)
        self.speed = float(v)
        self.microSec = 0
        self.angle = 0
    def translateInput(self, w):

        if (w >= 0):
            print("Turn Left")
            leftTurn = True;

        elif (w < 0):
            leftTurn = False
            print("Turn Right")

        self.turn(w, self.speed, leftTurn)

    def turn(self, w, v,leftTurn):

        l = 0.2
        x = 0.1
        r = v/w
        den = math.sqrt(r**2-x**2)
        a = math.atan(l/den)
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




