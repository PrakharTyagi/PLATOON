
import numpy
import math


class Translator:
    def __init__(self, w = 0, v = 0.86):
        self.omega = float(w)
        self.speed = float(v)
        self.speedMicro = 1500;
        self.microSec = 0
        self.angle = 0
        self.listOfRightKeys = [0, 0.7172, 0.7849, 0.8633,1.1609,1.3933,99999]
        self.listOfLeftKeys= [0, 1.0417, 1.0422, 1.1382,1.4199,1.3138,99999]

        self.rdict = {0.7172:1200, 0.7849:1250, 0.8633:1300,1.1609:1350,1.3933:1400,0: 1200, 99999:1500}
        self.ldict = {1.0417:1800, 1.0422:1750, 1.1382:1700,1.4199:1650,1.3138:1600,0: 1800, 99999:1500}

        # List containing measurements of rpms and resulting speeds.
        self.speeds = [[1200, 3.0], [1250, 2.1], [1400, 1.2], [1300, 1.7],
                    [1450, 0.78], [1500, 0]]
        self.speeds.sort(key = lambda x: x[1])

        self.throttle_min = 1000    # Minimum value for speedMicro.
        self.throttle_max = 1500    # Maximum value.


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


    def translate_speed(self, v):
        """Calculates the rpm value corresponding to the desired speed v.
        Interpolates linearly from a list of measurements. """
        length = len(self.speeds)

        if length < 2:
            print('Not enough measurements to translate speed input.')
            self.speedMicro = 1500
            return

        # Find the lowest index for which v is smaller than the speed value.
        i = 0
        while i < length and v > self.speeds[i][1]:
            i += 1

        # Get the lower and upper indices that will be used for line equation.
        if i <= 0:
            lower = 0
            upper = 1
        elif i >= length:
            lower = length - 2
            upper = length - 1
        else:
            lower = i - 1
            upper = i

        # Calculate speedMicro using straight line equation.
        k = (self.speeds[upper][0] - self.speeds[lower][0]) / (
            self.speeds[upper][1] - self.speeds[lower][1])
        self.speedMicro = self.speeds[lower][0] + (v - self.speeds[lower][1])*k

        # Make sure that the translated speed is within the bounds.
        if self.speedMicro > self.throttle_max:
            self.speedMicro = self.throttle_max
        if self.speedMicro < self.throttle_min:
            self.speedMicro = self.throttle_min


    def getMicroSec(self):
        return float(self.microSec)

    def getAngle(self):
        return self.angle

    def getSpeed(self):
        return self.speedMicro

    def getGear(self):
        return 60
