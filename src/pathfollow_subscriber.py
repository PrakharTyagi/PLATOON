#!/usr/bin/env python

import rospy
import path
import translator
from modeltruck_platooning.msg import *
import math
import struct
import sys
import socket
import time


class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, V, Ts, kp, ki, kd, const_vel, sum_limit,
                address = ('192.168.1.193', 2390), node_name = 'controller_sub',
                topic_name = 'truck2', topic_type = truckmocap):
        self.node_name = node_name
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.address = address

        self.V = V
        self.Ts = Ts
        self.k_p = kp
        self.k_i = ki
        self.k_d = kd
        self.const_vel = const_vel

        self.vel0 = 1500            # Truck at zero velocity.
        self.ang0 = 1500            # Neutral wheel angle.
        self.gr0 = 60               # First gear
        self.sumy = 0               # Accumulated error.
        self.sum_limit = sum_limit  # Limit for accumulated error.

        rospy.init_node(self.node_name, anonymous = True)
        rospy.Subscriber(self.topic_name, self.topic_type, self.callback)

        self.pt = path.Path()
        self.translator = translator.Translator(0, self.V)

        self.seqNum = 0
        self.packer = struct.Struct('<IIHhhh')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.1)

        self.send_data(self.address, self.vel0, self.ang0, self.gr0, True)


    def callback(self, data):
        """Called when the subscriber receives data. """
        omega = self.get_omega(data)
        self.translator.turn(omega, self.V)
        angle = int(self.translator.microSec)
        print('{}, {:.4f}'.format(angle, omega))

        self.send_data(self.address, self.const_vel, angle, self.gr0)


    def send_data(self, address, velocity, angle, gear, firstPack = False):
        """Sends data to the truck. """
        if firstPack:
            ms = 0xFFFFFFFF
            ns = 0xFFFFFFFF
            self.seqNum = 0xFFFF

        else:
            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10**9))
            self.seqNum = (self.seqNum + 1) % 0xFFFF

        command_msg = self.packer.pack(*(
                ms,  ns, self.seqNum, velocity, angle, gear))

        self.client_socket.sendto(command_msg, address)


    def get_omega(self, data):
        """Calculate the control input omega. """
        x = data.x
        y = data.y
        yaw = data.yaw

        index, closest = self.pt.get_closest([x, y])

        ey = self.pt.get_ey([x, y])     # y error (distance from path)
        self.sumy = self.sumy + ey      # Accumulated error.
        if self.sumy > self.sum_limit:
            self.sumy = self.sum_limit
        if self.sumy < -self.sum_limit:
            self.sumy = -self.sum_limit

        #print('Ctrl error: {:.4f}, sum error: {:.4f}'.format(ey, self.sumy))

        gamma = self.pt.get_gamma(index)
        gamma_p = self.pt.get_gammap(index)
        gamma_pp = self.pt.get_gammapp(index)

        cos_t = math.cos(yaw - gamma)     # cos(theta)
        sin_t = math.sin(yaw - gamma)     # sin(theta)

        yp = math.tan(yaw - gamma)*(1 - gamma_p*ey)*self.sign( # y prime
            self.V*cos_t/(1 - gamma_p*ey))

        u = - self.k_p*ey - self.k_d*yp - self.k_i * self.sumy # PID controller.

        omega = self.V*cos_t/(1 - gamma_p*ey) * (u*cos_t**2/(1 - gamma_p*ey) +
                                gamma_p*(1 + sin_t**2) +
                                gamma_pp*ey*cos_t*sin_t/(1 - gamma_p*ey))

        return omega


    def stop_truck(self):
        self.send_data(self.address, self.vel0, self.ang0, self.gr0)
        self.send_data(self.address, self.vel0, self.ang0, self.gr0)
        self.send_data(self.address, self.vel0, self.ang0, self.gr0)
        print('\nStopped truck')


    def sign(self, x):
        if x > 0:
            return 1
        else:
            return -1


def main():
    ax = 1.6    # Ellipse x-radius.
    ay = 1.2    # Ellipse y-radius.
    pts = 300   # Number of points on ellipse path.

    V = 0.6             # Velocity used by translator model.
    const_vel = 1460    # Constant velocity that is sent to the arduino.
    Ts = 0.05           # Sampling time (UNUSED).

    # PID parameters.
    # 0.5, 0.01, 3, limit 0.05 ok
    #0.7, -0.01 7 5000
    #translator2: 0.5, -0.02, 3, 5000, V = 0.6
    k_p = 0.5
    k_i = -0.02
    k_d = 3
    sum_limit = 5000    # Limit in accumulated error for I part of PID.

    center = [0.3, -0.5]

    address = ('192.168.1.193', 2390)

    # Subscriber info.
    node_name = 'controller_sub'
    topic_name = 'truck2'
    topic_type = truckmocap

    controller = Controller(V, Ts, k_p, k_i, k_d, const_vel, sum_limit,
        address = address, node_name = node_name,
        topic_name = topic_name, topic_type = topic_type)

    controller.pt.gen_circle_path([ax, ay], pts, center = center)  # Create reference path.

    rospy.spin()

    controller.stop_truck()   # Stop truck after subscriber has terminated.


if __name__ == '__main__':
    main()
