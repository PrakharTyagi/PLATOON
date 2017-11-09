#!/usr/bin/env python

import rospy
from modeltruck_platooning.msg import drive_param
from mocap_source_2 import Mocap

from socket import *
import struct
import time

import sys

seqNum = 0

firstPack = True

packer = struct.Struct('<IIHhhh') # Format: <timestamp ms> <timestamp us> <seqNum> <velocity> <angle> <gear>

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

        return x, y, yaw



def receiver(vehicle_id):
    # global seqNum, firstPack
    # velocity = 1500
    # angle = 1500
    # gear = 60   #first gear

    mytruck = Truck()

    # vel = velocity
    # ang = angle
    # gr = gear

    if vehicle_id == 1:
        address = ('192.168.1.194', 2390)

    elif vehicle_id == 2:
        address = ('192.168.1.193', 2390)

    # client_socket = socket(AF_INET, SOCK_DGRAM)
    # client_socket.settimeout(0.1)

    # ms = 0xFFFFFFFF # Send all F in first pack to notify data-reset
    # ns = 0xFFFFFFFF
    # seqNum = 0xFFFF
    # firstPack = False

    while True:
        inpt = eval(raw_input('\nEnter\n'))


        if inpt == 0:
	    	break
        if inpt == 1:
            x, y, yaw = mytruck.get_values()
            print('\nx: {}\ny: {}\nyaw: {}'.format(x, y, yaw))
        #if inpt == 2:
        #    print('\nVelocity: {}\nAngle: {}\nGear: {}'.format(vel, ang, gr))


        #t = time.time()
        #ms = int(t)
        #ns = int((t % 1) * (10**9))

        #print('\nVelocity: {}\nAngle: {}\nGear: {}'.format(vel, ang, gr))

    	#command_msg = packer.pack(*(ms,  ns, seqNum, vel, ang, gr))
    	#client_socket.sendto(command_msg, address)

        #if inpt == 0:
        #    break



if __name__ == '__main__':
    try:
        receiver(int(sys.argv[1])) # argv[1] which truck it is.
    except:
        pass
