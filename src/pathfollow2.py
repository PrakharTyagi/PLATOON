#!/usr/bin/env python

from path import Path
from translator import Translator
#from sim_truck import Truck
import time
import matplotlib.pyplot as plt
import math
from socket import *
import struct
import sys
from modeltruck_platooning.srv import *
import rospy


def sign(x):
    if x > 0:
        return 1
    else:
        return -1

def get_omega(pt, response, V, tid, sumy):
    # Circle r 1 k = 0.005 others 0 ok.
    k_py = 0.5
    k_vy = 0.3
    k_i = 0.001

    sum_limit = 0.05

    x = response.x
    y = response.y
    yaw = response.yaw
    index, closest = pt.get_closest([x,y])

    ey = pt.get_ey([x, y])
    sumy = sumy + ey*tid
    if sumy > sum_limit:
        sumy = sum_limit
    if sumy < -sum_limit:
        sumy = -sum_limit
    print('sum: {:.4f}, error {:.4f}'.format(sumy, ey))

    gamma = pt.get_gamma(index)
    gamma_p = pt.get_gammap(index)
    gamma_pp = pt.get_gammapp(index)

    cos_t = math.cos(yaw-gamma)     # cos(theta)
    sin_t = math.sin(yaw-gamma)     # sin(theta)
    # den = 1/(1-gamma_p*ey)          # 1/(1-gamma_p*y)
    # cos_den = cos_t*den             # cos/(1-gamma_p*y)

    yp = math.tan(yaw - gamma)*(1 - gamma_p*ey)*sign(V*cos_t/(1 - gamma_p*ey))
    u = - k_py*ey - k_vy*yp - k_i * sumy
    omega = V*cos_t/(1 - gamma_p*ey) * (u*cos_t**2/(1 - gamma_p*ey) +
                            gamma_p*(1 + sin_t**2) +
                            gamma_pp*ey*cos_t*sin_t/(1 - gamma_p*ey))



    return omega, sumy


def main():
    ax = 1.3
    ay = 1.3
    pts = 300
    V = 1
    tid = 0.05

    l = 0.25

    init_velocity = 1500    # Standstill values for stopping the truck.
    init_angle = 1500
    init_gear = 60

    vel = 1460          # Constant velocity used by truck.
    ang = init_angle
    gr = init_gear

    service_name = 'test_plot'
    service_type = TestPlot

    try:
        rospy.wait_for_service(service_name, timeout = 2)
        srv_handle = rospy.ServiceProxy(service_name, service_type)
    except Exception as e:
        print('Service connection failed: {}'.format(e))


    pt = Path()
    pt.gen_circle_path([ax, ay], pts)

    translator = Translator(0, V)

    seqNum = 0
    firstPack = True
    packer = struct.Struct('<IIHhhh') # Format: <timestamp ms> <timestamp us> <seqNum> <velocity> <angle> <gear>
    address = ('192.168.1.193', 2390)

    client_socket = socket(AF_INET, SOCK_DGRAM)
    client_socket.settimeout(0.1)

    ms = 0xFFFFFFFF # Send all F in first pack to notify data-reset
    ns = 0xFFFFFFFF
    seqNum = 0xFFFF
    command_msg = packer.pack(*(ms,  ns, seqNum, vel, ang, gr))
    client_socket.sendto(command_msg, address)
    firstPack = False


    sumy = 0
    ang = 1500
    while True:
        try:

            response = srv_handle()
            omega, sumy = get_omega(pt, response, V, tid, sumy)

            if omega < 2*V/l:
                translator.translateInput(omega)
                ang = translator.getMicroSec()

            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10**9))

            command_msg = packer.pack(*(ms,  ns, seqNum, vel, ang, gr))
            client_socket.sendto(command_msg, address)

            time.sleep(tid)

        except KeyboardInterrupt:
            command_msg = packer.pack(
                            *(ms, ns, seqNum, init_velocity, init_angle,
                            init_gear))
            client_socket.sendto(command_msg, address)
            break

    command_msg = packer.pack(
                    *(ms, ns, seqNum, init_velocity, init_angle,
                    init_gear))
    client_socket.sendto(command_msg, address)

if __name__ == '__main__':
    main()
