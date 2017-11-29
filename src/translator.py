import math

class Translator:
    """Class for translating truck wheel angle, speed, and angular velocity
    to the corrseponding pwm values. """
    def __init__(self):
        self.speed_pwm = 1500   # pwm value for the speed.
        self.alpha_pwm = 1500   # pwm value for the wheel angle alpha.

        # Turning radii and their corresponding pwms for left and right turns.
        self.rdict = {0.7172:1200, 0.7849:1250, 0.8633:1300, 1.1609:1350,
            1.3933:1400, 20:1500, 3:1450}
        self.ldict = {1.0417:1800, 1.0422:1750, 1.1382:1700, 1.4199:1650,
            1.7138:1600, 20:1500, 3:1550}

        self.listOfRightKeys = sorted(self.rdict, key = self.rdict.get)
        self.listOfLeftKeys = sorted(self.ldict, key = self.ldict.get,
            reverse = True)

        # List containing measurements of speed pwms and resulting speeds.
        self.speeds = [[1500, 0], [1450, 0.89], [1400, 1.97], [1350, 2.75]]
        self.speeds.sort(key = lambda x: x[1])

        self.speed_pwm_min = 1100    # Minimum value for speed_pwm.
        self.speed_pwm_max = 1500    # Maximum value for speed_pwm.

        self.l = 0.27                   # Distance between wheel pairs.
        self.alpha_min = - math.pi/6    # Minimum wheel turning angle.
        self.alpha_max = math.pi/6      # Maximum wheel turning angle.
        self.alpha_pwm_min = 1100       # Minimum wheel angle pwm.
        self.alpha_pwm_max = 1900       # Maximum wheel angle pwm.

        # Calculate a list of pairs that maps wheel angle pwms to wheel angles.
        self.alphas = self._calc_alphas()


    def turn(self, w, v=1.00000 ):
        l = float(0.25)
        x = float(l/2)
        if w == 0:
            self.alpha_pwm = 1500
            return

        r = float(v/w)
        r = abs(r)
        if(w>=0):
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
                break
            else:
                prevElem = elem
        if not idxRange:
            microSum = 1500
        else:
            y1 = dict[radList[idxRange[0]]]
            y2 = dict[radList[idxRange[1]]]
            x1 = radList[idxRange[0]]
            x2 = radList[idxRange[1]]
            if(x2 == 99999):
                microSum = dict[99999]
            else:
                k = (y2-y1)/(x2-x1)
                microSum = k*(r-x1)+y1

        self.alpha_pwm = float(microSum)


    def _translate_speed(self, v):
        """Calculates the pwm value corresponding to the desired speed v.
        Interpolates linearly from a list of measurements. """
        length = len(self.speeds)

        if length < 2:
            print('Not enough measurements to translate speed input.')
            self.speed_pwm = 1500
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

        self.speed_pwm = int(
            self.speeds[lower][0] + (v - self.speeds[lower][1])*k)

        # Make sure that the translated speed is within the bounds.
        if self.speed_pwm > self.speed_pwm_max:
            self.speed_pwm = self.speed_pwm_max
        if self.speed_pwm < self.speed_pwm_min:
            self.speed_pwm = self.speed_pwm_min


    def _translate_alpha(self, alpha):
        """Calculates the pwm value corresponding to the desired wheel angle.
        Interpolates linearly from a list of measurements. """
        length = len(self.alphas)

        if length < 2:
            print('Not enough measurements to translate speed input.')
            self.alpha_pwm = 1500
            return

        # Find the lowest index for which alpha is smaller than the angle value.
        i = 0
        while i < length and alpha > self.alphas[i][1]:
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

        # Calculate alpha_pwm using straight line equation.
        k = (self.alphas[upper][0] - self.alphas[lower][0]) / (
            self.alphas[upper][1] - self.alphas[lower][1])

        self.alpha_pwm = int(
            self.alphas[lower][0] + (alpha - self.alphas[lower][1])*k)

        # Make sure that the translated speed is within the bounds.
        if self.alpha_pwm > self.alpha_pwm_max:
            self.alpha_pwm = self.alpha_pwm_max
        if self.alpha_pwm < self.alpha_pwm_min:
            self.alpha_pwm = self.alpha_pwm_min


    def _calc_alphas(self):
        """Returns a list of pairs with alpha values and corresponding pwms. """
        alphas_list = []

        # Translate values for right turn.
        for x in self.listOfRightKeys:
            if x > self.l/2:
                alpha = - math.atan(self.l/math.sqrt(x**2 - (self.l/2)**2))
            else:
                alpha = self.alpha_min

            alphas_list.append([self.rdict[x], alpha])

        # Translate values for left turn.
        for x in self.listOfLeftKeys:
            if x > self.l/2:
                alpha = math.atan(self.l/math.sqrt(x**2 - (self.l/2)**2))
            else:
                alpha = self.alpha_max

            alphas_list.append([self.ldict[x], alpha])

        alphas_list.sort(key = lambda x: x[1]) # Sort ascending by alpha value.

        return alphas_list


    def get_speed(self, v):
        """Returns the pwm speed that corresponds to the speed v. """
        self._translate_speed(v)
        return self.speed_pwm


    def get_angle(self, w, v):
        """Returns the pwm angle that corresponds to given speed v and angular
        velocity w. """
        self.turn(w, v)
        return self.alpha_pwm


    def get_angle_from_alpha(self, alpha):
        """Returns the pwm angle that corresponds to wheel angle alpha. """
        self._translate_alpha(alpha)
        return self.alpha_pwm


    def getAngle(self):
        return self.alpha_pwm


    def getSpeed(self):
        return self.speed_pwm


    def getGear(self):
        return 60



# Old/original turning radius dicts.
# self.rdict = {0.7172:1200, 0.7849:1250, 0.8633:1300,1.1609:1350,1.3933:1400,0: 1200, 99999:1500}
# self.ldict = {1.0417:1800, 1.0422:1750, 1.1382:1700,1.4199:1650,1.3138:1600,0: 1800, 99999:1500}
