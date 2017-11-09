#!/usr/bin/env python

import rospy
from modeltruck_platooning.msg import drive_param
from mocap_source_2 import Mocap

from socket import *
import struct
import time
import math

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
    global seqNum, firstPack
    init_velocity = 1500# Initial values. Zero speed and angle.
    init_angle = 1500
    init_gear = 60      # first gear

    ang_max = 1900      # Maximum angle
    ang_min = 1100
    Ts = 0.25           # Update interval in seconds
    r_ref = 1           # Circle reference radius
    k = 400             # Constant in P controller
    const_vel = 1460    # Constant velocity used

    if vehicle_id == 1:
        address = ('192.168.1.194', 2390)
    elif vehicle_id == 2:
        address = ('192.168.1.193', 2390)

    mytruck = Truck()   # Mocap truck object

    vel = const_vel     # Values that are updated and sent to truck.
    ang = init_angle
    gr = init_gear

    client_socket = socket(AF_INET, SOCK_DGRAM)
    client_socket.settimeout(0.1)

    ms = 0xFFFFFFFF # Send all F in first pack to notify data-reset
    ns = 0xFFFFFFFF
    seqNum = 0xFFFF
    firstPack = False

    try:
        # Loop until interrupted.

        while True:

            x, y, yaw = mytruck.get_values() # Get truck data from mocap.
            x = 1;
            y = 1;

            r = math.sqrt(x^2 + y^2)     # Distance from center
            e = r_ref - r           # Distance error

            # Controller (if truck outside ref circle, ang < 1500)
            ang = int(init_angle + k*e)

            if ang > ang_max:        # Don't go outside angle limits.
                ang = ang_max
            if ang < ang_min:
                ang = ang_min

            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10**9))

            command_msg = packer.pack(*(ms,  ns, seqNum, vel, ang, gr))
            client_socket.sendto(command_msg, address)

            time.sleep(Ts)

    except KeyboardInterrupt:
        # On ctrl-C reset the speed ang angle to zero.
        command_msg = packer.pack(
                        *(ms, ns, seqNum, init_velocity, init_angle, init_gear))
        client_socket.sendto(command_msg, address)
        print('Interrupted')



if __name__ == '__main__':
    try:
        receiver(int(sys.argv[1])) # argv[1] which truck it is.
    except:
        pass
