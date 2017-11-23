
import numpy
import math


class Translator:
    def __init__(self, w = 0, v = 0.86):
        self.omega = float(w)
        self.speed = float(v)
        self.speedMicro = 1500;
        self.microSec = 0
        self.angle = 0
        self.listOfLeftKeys = [0, 0.7172, 0.7849, 0.8633,1.1609,1.3933,99999]
        self.listOfRightKeys= [0, 1.0417, 1.0422, 1.1382,1.4199,1.3138,99999]

        self.ldict = {0.7172:1200, 0.7849:1250, 0.8633:1300,1.1609:1350,1.3933:1400,0: 1200, 99999:1500}
        self.rdict = {1.0417:1800, 1.0422:1750, 1.1382:1700,1.4199:1650,1.3138:1600,0: 1800, 99999:1500}


    def translateInput(self, w):
        if (w >= 0):
            #print("Turn Left")
            leftTurn = True;

        elif (w < 0):
            leftTurn = False
            #print("Turn Right")

        #self.turn(w, self.speed, leftTurn)
        print(w)
        return leftTurn

    def turn(self, w, v=1.00000 ):

        l = float(0.25)
        x = float(l/2)
        r = float(v/w)
        r = abs(r)
        if(w>=0):
        #if leftTurn == True:
            radList = self.listOfLeftKeys
            dict    =self.ldict

        else:
            radList = self.listOfRightKeys
            dict = self.rdict

        prevElem = 0
        idxRange = []
        for idx, elem in enumerate(radList):
            if (r > prevElem) & (r<elem):
                idxRange = [idx-1,idx]
                #print(elem)
                break
            else:
               # print(elem)
                #print(r)
                prevElem = elem
        if not idxRange:
            #print("empty")
            microSum = 1500
        else:
            y1 = dict[radList[idxRange[0]]]
            y2 = dict[radList[idxRange[1]]]
            x1 = radList[idxRange[0]]
            x2 = radList[idxRange[1]]
            if(x2 == 99999):
                microSum = dict[99999]
               # print microSum
            else:
                k = (y2-y1)/(x2-x1)
                microSum = k*(r-x1)+y1
                #print(x1)
                #print(x2)
               # print(microSum)
        self.microSec = microSum
    def getMicroSec(self):
        return float(self.microSec)

    def getAngle(self):
        return self.angle

    def getSpeed(self):
        return self.speedMicro

    def getGear(self):
        return 60
