#!/usr/bin/env python

import rospy
#from f1tenth_task1.msg import drive_values
#from f1tenth_task1.msg import drive_param
from platoon.msg import drive_param


from socket import *
import struct
import time

import sys


"""
 1. Subscribe to the keyboard messages (If you use the default keyboard.py, you must subcribe to "drive_paramters" which is publishing messages of "drive_param")
 2. Map the incoming values to the needed PWM values
 3. Publish the calculated PWM values on topic "drive_pwm" using custom message "drive_values"
"""




velocity = 1500
angle = 1500
gear = 60   #first gear

seqNum = 0

firstPack = True

packer = struct.Struct('<IIHhhh') # Format: <timestamp ms> <timestamp us> <seqNum> <velocity> <angle> <gear>

def get_commands(data):
    global velocity,angle,gear

    velocity = data.velocity
    angle = data.angle
    gear = data.gear

def receiver(vehicle_id):
    global velocity, angle, seqNum, firstPack
    if vehicle_id == 1:
        address = ('192.168.1.194', 2390)

        rospy.init_node('key_receiver1', anonymous = True)
        rospy.Subscriber("/vehicle_1/drive_parameters", drive_param, get_commands)

    elif vehicle_id == 2:
        address = ('192.168.1.193', 2390)
        rospy.init_node('key_receiver2', anonymous = True)
        rospy.Subscriber("/vehicle_2/drive_parameters", drive_param, get_commands)

    client_socket = socket(AF_INET, SOCK_DGRAM)
    client_socket.settimeout(0.1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if firstPack:
            ms = 0xFFFFFFFF # Send all F in first pack to notify data-reset
            ns = 0xFFFFFFFF
            seqNum = 0xFFFF
            firstPack = False
        else:
            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10**9))
        command_msg = packer.pack(*(ms,  ns, seqNum, velocity, angle, gear))
        client_socket.sendto(command_msg, address)
        seqNum = (seqNum + 1) % 0xFFFF
        rate.sleep()


if __name__ == '__main__':
    try:
        receiver(int(sys.argv[1]))
    except rospy.ROSInterruptException:
        pass
