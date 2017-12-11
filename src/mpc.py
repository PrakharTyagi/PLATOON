#!/usr/bin/env python

import controllerGUI


import mpcfrenet
import ampcfrenet

from platoon.msg import truckmocap
from platoon.msg import truckcontrol

import rospy
import sys


def main(args):


    address1 = ('192.168.1.194', 2390)
    address2 = ('192.168.1.193', 2390)

    address = address2   # Truck address.
    truck_id = 2
    try:
        if int(args[1]) == 1:
                address = address1   # Truck address.
                truck_id = 1
    except:
        pass

    # Information for controller subscriber.
    node_name = 'controller_sub'
    mocap_topic_name = 'truck_topic'
    mocap_topic_type = truckmocap

    truck_topic_name = 'truck_control'
    truck_topic_type = truckcontrol

    # Data for controller reference path.
    x_radius = 1.6
    y_radius = 1.1
    center = [0.3, -1.3]

    # Controller tuning variables.
    v = 0.8             # Speed of the truck.

    v_ref = 0.8

    Ts = 0.05


    mpc = mpcfrenet.Controller(address, node_name, mocap_topic_type,
        mocap_topic_name, truck_topic_type, truck_topic_name, Ts = Ts,
        xyr = [x_radius, y_radius], xyc = center, v_ref = v_ref,
        truck_id = truck_id)

    ampc = ampcfrenet.Controller(address, node_name, mocap_topic_type,
        mocap_topic_name, truck_topic_type, truck_topic_name, Ts = Ts,
        xyr = [x_radius, y_radius], xyc = center, v_ref = v_ref,
        truck_id = truck_id)

    ctrl_gui = controllerGUI.ControllerGUI(ampc)


if __name__ == '__main__':
    main(sys.argv)
