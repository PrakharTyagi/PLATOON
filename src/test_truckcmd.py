#!/usr/bin/env python

# File to test values of velocity, angle and speed. Program runs and prompts
# for values on the form ang vel gear. Can enter only one, two or all. Sends to
# truck when values are entered. Run with arg 1 or 2 depending on truck.

import rospy
from modeltruck_platooning.msg import drive_param

from socket import *
import struct
import time

import sys

seqNum = 0

firstPack = True

packer = struct.Struct('<IIHhhh') # Format: <timestamp ms> <timestamp us> <seqNum> <velocity> <angle> <gear>

def receiver(vehicle_id):
    global seqNum, firstPack
    init_velocity = 1500
    init_angle = 1500
    init_gear = 60   #first gear

    vel = init_velocity
    ang = init_angle
    gr = init_gear

    if vehicle_id == 1:
        address = ('192.168.1.193', 2390)
    elif vehicle_id == 2:
        address = ('192.168.1.194', 2390)

    client_socket = socket(AF_INET, SOCK_DGRAM)
    client_socket.settimeout(0.1)

    ms = 0xFFFFFFFF # Send all F in first pack to notify data-reset
    ns = 0xFFFFFFFF
    seqNum = 0xFFFF
    firstPack = False

    try:
        while True:
            # Enter only velocity, velocity and angle or all three. Empty
            # will reset the values and quit the program.
            inpt = raw_input('\nEnter velocity angle gear\n')

            if len(inpt) != 0:

                inpt_list = inpt.split(' ')
                vel = int(inpt_list[0])

                if len(inpt_list) > 1:

                    ang = int(inpt_list[1])


                    if len(inpt_list) > 2:
                        gr = int(inpt_list[2])

                print(
                    '\nVelocity: {}\nAngle: {}\nGear: {}'.format(vel, ang, gr))

                t = time.time()
                ms = int(t)
                ns = int((t % 1) * (10**9))

                command_msg = packer.pack(*(ms,  ns, seqNum, vel, ang, gr))
                client_socket.sendto(command_msg, address)

            else:

                # Reset truck values and quit loop.
                command_msg = packer.pack(
                                *(ms, ns, seqNum, init_velocity, init_angle,
                                init_gear))
                client_socket.sendto(command_msg, address)
                break

    except KeyboardInterrupt:
        # Reset the values on ctrl-C.
        command_msg = packer.pack(
                        *(ms, ns, seqNum, init_velocity, init_angle, init_gear))
        client_socket.sendto(command_msg, address)
        print('\nresetting')



if __name__ == '__main__':
    try:
        receiver(int(sys.argv[1])) # argv[1] which truck it is.
    except:
        pass
