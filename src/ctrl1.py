#!/usr/bin/env python

import controllerGUI
import controller1

from platoon.msg import truckmocap

import rospy
import sys

def main(args):
    address = ('192.168.1.193', 2390)   # Truck address.
    truck_id = 2
    try:
        if int(args[1]) == 1:
                address = ('192.168.1.194', 2390)   # Truck address.
                truck_id = 1
    except:
        print('e')
        pass

    # Information for controller subscriber.
    node_name = 'controller_sub'
    topic_name = 'truck_topic'
    topic_type = truckmocap

    # Data for controller reference path.
    x_radius = 1.6
    y_radius = 1.2
    center = [0.3, -0.5]

    # Controller tuning variables.
    v = 0.89             # Speed of the truck.

    k_p = 0.5
    k_i = -0.02
    k_d = 3
    sum_limit = 5000    # Limit in accumulated error for I part of PID.

    # Initialize controller and GUI.
    controller = controller1.Controller(
        address, node_name, topic_type, topic_name,
        v = v, k_p = k_p, k_i = k_i, k_d = k_d, sum_limit = sum_limit,
        truck_id = truck_id)
    controller.set_reference_path([x_radius, y_radius], center)

    ctrl_gui = controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
