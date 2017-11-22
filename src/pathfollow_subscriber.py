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
    def __init__(self, V, Ts, kp, ki, kd, const_vel, sum_limit):
        self.node_name = 'controller_sub'
        self.topic_name = 'truck2'
        self.topic_type = truckmocap

        self.address2 = ('192.168.1.193', 2390)

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

        self.send_data(self.address2, self.vel0, self.ang0, self.gr0, True)


    def callback(self, data):
        """Called when the subscriber receives data. """
        omega = self.get_omega(data)
        self.translator.translateInput(omega)
        angle = int(self.translator.getMicroSec())

        self.send_data(self.address2, self.const_vel, angle, self.gr0)


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

        print('Error: {:.4f}'.format(ey))

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
        self.send_data(self.address2, self.vel0, self.ang0, self.gr0)
        self.send_data(self.address2, self.vel0, self.ang0, self.gr0)
        self.send_data(self.address2, self.vel0, self.ang0, self.gr0)
        print('\nStopped truck')


    def sign(self, x):
        if x > 0:
            return 1
        else:
            return -1


def main():
    ax = 1.2    # Ellipse x-value.
    ay = 1.2    # Ellipse y value.
    pts = 300   # Number of points on ellipse path.

    V = 0.6     # Velocity (used by translator model, not arduino).
    Ts = 0.05   # Sampling time (UNUSED).

    # PID parameters.
    k_p = 0.5
    k_i = 0.001
    k_d = 0.3
    sum_limit = 0.05    # Limit in accumulated error for I part of PID.

    const_vel = 1460    # Constant velocity that is sent to the arduino.

    ctrl = Controller(V, Ts, k_p, k_i, k_d, const_vel, sum_limit)
    ctrl.pt.gen_circle_path([ax, ay], pts)  # Create reference path.

    rospy.spin()

    ctrl.stop_truck()   # Stop truck after subscriber has terminated.


if __name__ == '__main__':
    main()
