#!/usr/bin/env python

import rospy
from platoon.msg import *

import sys
import struct
import socket
import time


class DataSender():
    """Class for sending data to the trucks. Assumes there are so many available
    trucks to send to as there are addresses. The addresses will correspond
    to truck_id 1, 2, etc. """
    def __init__(self, node_name, topic_type, topic_name, addresses,
        print_info = False):

        self.addresses = addresses
        self.seqNums = [0xFFFF for i in range(len(addresses))]
        self.print_info = print_info

        rospy.init_node(node_name, anonymous = True)
        rospy.Subscriber(topic_name, topic_type, self._callback)

        # For sending data.
        self.seqNum = 0
        self.packer = struct.Struct('<IIHhhh')
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.client_socket.settimeout(0.1)

        self._send_first()

        rospy.spin()


    def _callback(self, data):
        """Receives data from the topic and sends it to the right truck. """
        self._send_data(data.truck_id, data.angle, data.speed)


    def _send_first(self):
        """Sends the first data packet. """
        for i, x in enumerate(self.addresses):
            self._send_data(i + 1, 1500, 1500, True)


    def _send_data(self, truck_id, angle, speed, first = False):
        """Sends angle, speed to truck truck_id. """
        # Get the address of the truck corresponding to the truck_id.
        try:
            address = self.addresses[truck_id - 1]
        except:
            print('Invalid truck ID.')
            return

        # Probably not necessary stuff.
        if first:
            ms = 0xFFFFFFFF
            ns = 0xFFFFFFFF
        else:
            self.seqNums[truck_id - 1] = \
                (self.seqNums[truck_id - 1] + 1) % 0xFFFF

            t = time.time()
            ms = int(t)
            ns = int((t % 1) * (10**9))
            self.seqNum = (self.seqNum + 1) % 0xFFFF
        seqNum = self.seqNums[truck_id - 1]

        # Pack message and send to address.
        command_msg = self.packer.pack(*(
                ms,  ns, self.seqNum, speed, angle, 60))

        self.client_socket.sendto(command_msg, address)
        if self.print_info:
            print('Sending angle {:.0f}, speed {:.0f} to {}'.format(
            angle, speed, address[0]))


def main(args):
    topic_name = 'truck_control'
    topic_type = truckcontrol
    node_name = 'sender_sub'

    address1 = ('192.168.1.194', 2390)
    address2 = ('192.168.1.193', 2390)

    addresses = [address1, address2]

    print_info = False
    try:
        if int(args[1]) == 1:
            print_info = True
    except:
        pass

    datasender = DataSender(node_name, topic_type, topic_name,
        addresses, print_info)


if __name__ == '__main__':
    main(sys.argv)
