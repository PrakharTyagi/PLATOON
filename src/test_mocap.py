#!/usr/bin/env python

# Short program to test if it can read mocap data of the truck.

import rospy
from modeltruck_platooning.msg import drive_param
from mocap_source_2 import Mocap
from socket import *
import struct
import time
import sys
import math

class Truck:
    def __init__(self):
        #initialize mocap connection
        self.mocap = Mocap(host = '192.168.1.199', info = 1)
        self.mocap_body1 = self.mocap.get_id_from_name('TruckVehicle2')

    def get_values(self):
        truck_state1 = self.mocap.get_body(self.mocap_body1)
        x = truck_state1['x']
        y = truck_state1['y']
        yaw = truck_state1['yaw']

        return x, y, yaw*math.pi/180


def receiver(vehicle_id):

    mytruck = Truck()

    while True:
        inpt = eval(raw_input('\nEnter 1 to see coordinates\n'))

        if inpt == 0:
            break
        if inpt == 1:
            x, y, yaw = mytruck.get_values()
            print('\nx: {}\ny: {}\nyaw: {}'.format(x, y, yaw))

if __name__ == '__main__':
    try:
        receiver()
    except:
        pass
