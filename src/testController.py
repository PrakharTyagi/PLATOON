from controller import Controller
import math

ctrl = Controller()

ctrl.translateInput(10.001)

a = ctrl.getMicroSec()
b = ctrl.getAngle()
#print("New angle = ",a(2) ," = ", a(2)*180/math.pi)
print("New input = ", a)
print("Angle = ", b)

