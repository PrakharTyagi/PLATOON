#!/usr/bin/env python

# Creates controller1 and a GUI for it. Choose truck 1 or 2 by entering the
# number as argument when running the script.

import controllerGUI
import controller_onetruck

from platoon.msg import truckmocap
from platoon.msg import truckcontrol

import rospy
import sys

def main(args):
    truck_id = 2
    try:
        if int(args[1]) == 1:
                truck_id = 1
    except:
        pass

    # Information for controller subscriber.
    node_name = 'controller_sub'
    topic_name = 'truck_topic'
    topic_type = truckmocap

    truck_topic_name = 'truck_control'
    truck_topic_type = truckcontrol

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0.3, -1.3]

    # Controller tuning variables.
    v = 0.89             # Speed of the truck.

    k_p = 0.5
    k_i = -0.02
    k_d = 3

    # Initialize controller and GUI.
    controller = controller_onetruck.Controller(
        node_name, topic_type, topic_name,
        truck_topic_type, truck_topic_name,
        v = v, k_p = k_p, k_i = k_i, k_d = k_d,
        truck_id = truck_id)
    controller.set_reference_path([x_radius, y_radius], center)

    ctrl_gui = controllerGUI.ControllerGUI(controller)


if __name__ == '__main__':
    main(sys.argv)
