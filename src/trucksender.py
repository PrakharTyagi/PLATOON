#!/usr/bin/env python
import struct
import socket
import time

class TruckSender():
    """Class for sending data to a specific truck. Needs the IP-address of the
    truck. """
    def __init__(self, address):
        self.address = address  # IP-address of the truck.

        # Initial values for which the truck is at standstill.
        self.init_velocity = 1500
        self.init_angle = 1500
        self.init_gear = 1
        self.gearvals = [60, 140, 220]
        self.init_gearval = self.gearvals[self.init_gear - 1]

        # Variables for saving which values that are sent to the truck.
        self.velocity = self.init_velocity
        self.angle = self.init_angle
        self.gearval = self.init_gearval

        # For sending data.
        self.seqNum = 0
        self.packer = struct.Struct('<IIHhhh')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.1)

        # Send initial data to the truck.
        self.send_data(self.init_velocity, self.init_angle,
            self.init_gearval, True)


    def send_data(self, velocity = None, angle = None, gear = None,
        firstPack = False):
        """Sends data to the truck. Sends previous values if none specified. """
        if firstPack:
            ms = 0xFFFFFFFF
            ns = 0xFFFFFFFF
            self.seqNum = 0xFFFF
        else:
            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10**9))
            self.seqNum = (self.seqNum + 1) % 0xFFFF

        if not velocity is None:
            self.velocity = velocity

        if not angle is None:
            self.angle = angle

        if not gear is None:
            if gear >= 1 and gear <= 3:
                self.gear = gear
                self.gearval = self.gearvals[self.gear - 1]

        command_msg = self.packer.pack(*(
                ms,  ns, self.seqNum, self.velocity, self.angle, self.gearval))

        self.client_socket.sendto(command_msg, self.address)


    def stop_truck(self):
        """Stops the truck by sending the initial values to it. """
        self.send_data(self.init_velocity, self.init_angle, self.init_gearval)
        self.send_data(self.init_velocity, self.init_angle, self.init_gearval)
        self.send_data(self.init_velocity, self.init_angle, self.init_gearval)
