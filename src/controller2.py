#!/usr/bin/env python

import rospy
import math
import sys

from platoon.msg import truckmocap
import path
import trucksender
import translator
import frenetpid

class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, address, node_name, topic_type, topic_name, freq = 20,
        v = 0, k_p = 0, k_i = 0, k_d = 0, sum_limit = 100,
        truck_id = 2):

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = ['k_p', 'k_i', 'k_d', 'v', 'sum_limit']

        self.v = v  # Velocity of the truck.
        self.angle_pwm = 1500

        # Radii and center for reference path ellipse.
        self.xr = 0
        self.yr = 0
        self.xc = 0
        self.yc = 0

        self.truck_id = truck_id

        self.running = False    # Controlling if controller is running or not.

        # Create reference path object, translator, and sender.
        self.pt = path.Path()
        self.translator = translator.Translator()
        self.sender = trucksender.TruckSender(address)
        self.frenet = frenetpid.FrenetPID(path, k_p, k_i, k_d, sum_limit, freq)

        self.v_pwm = self.translator.get_speed(self.v) # PWM velocity.

        # Setup subscriber node.
        rospy.init_node(node_name, anonymous = True)
        rospy.Subscriber(topic_name, topic_type, self._callback)

        print('\nController initialized. Truck {}.\n'.format(self.truck_id))


    def _callback(self, data):
        """Called when the subscriber receives data. """
        if self.truck_id == 2:
            x = data.x2
            y = data.y2
            yaw = data.yaw2
            vel = data.velocity2
        else:
            x = data.x1
            y = data.y1
            yaw = data.yaw1
            vel = data.velocity1

        timestamp = data.timestamp

        self._control(x, y, yaw, vel)


    def _control(self, x, y, yaw, vel):
        """Perform control actions from received data. Sends new values to
        truck. """
        if self.running:

            omega = self.frenet.get_omega(x, y, yaw, vel)
            self.angle_pwm = int(self.translator.get_angle(omega, vel))

            self.v_pwm = self.translator.get_speed(self.v) # pwm value.

            self.sender.send_data(self.v_pwm, self.angle_pwm)

            print('Control error: {:5.2f}'.format(self.frenet.get_y_error()))


    def stop(self):
        """Stops/pauses the controller. """
        self.sender.stop_truck(self.angle_pwm)
        if self.running:
            self.running = False
            print('Controller stopped.\n')


    def start(self):
        """Starts the controller. """
        if len(self.pt.path) == 0:
            print('Error: no reference path to follow.')
            return
        if not self.running:
            self.running = True
            print('Controller started.')


    def set_adjustables(self, values):
        """Used by the GUI to set the adjustable values. values is a list with
        the same size as the list returned by get_adjustables(). The values
        should here be treated in the same order as specified in that list. """
        try:
            k_p = float(values[0])
            k_i = float(values[1])
            k_d = float(values[2])
            v = float(values[3])
            sum_limit = float(values[4])

        except:
            print('\nInvalid control parameters entered.')
            return

        self.frenet.set_pid(k_p, k_i, k_d, sum_limit)
        self.frenet.reset_sum()
        self.v = v
        self.v_pwm = self.translator.get_speed(self.v)

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        return self.adjustables, [self.frenet.k_p, self.frenet.k_i,
            self.frenet.k_d, self.v, self.frenet.sum_limit]


    def set_reference_path(self, radius, center = [0, 0], pts = 400):
        """Sets a new reference ellipse path. """
        if isinstance(radius, list):
            if len(radius) > 1:
                self.xr = radius[0]
                self.yr = radius[1]
            else:
                self.xr = radius[0]
                self.yr = radius[0]
        else:
            self.xr = radius
            self.yr = radius

        self.xc = center[0]
        self.yc = center[1]
        self.pt.gen_circle_path([self.xr, self.yr], pts, [self.xc, self.yc])
        self.frenet.update_path(self.pt)


    def run(self):
        """Runs the controller. Needs to be called if not using the GUI. """
        self.start()
        rospy.spin()
        self.stop()


def main(args):
    address = ('192.168.1.193', 2390)   # Truck address.
    truck_id = 2
    try:
        if int(args[1]) == 1:
                address = ('192.168.1.194', 2390)   # Truck address.
                truck_id = 1
    except:
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
    v = 0.89             # Velocity used by translator model.

    k_p = 0.5
    k_i = -0.02
    k_d = 3
    sum_limit = 5000    # Limit in accumulated error for I part of PID.

    # Initialize controller.
    controller = Controller(address, node_name, topic_type, topic_name,
        v = v, k_p = k_p, k_i = k_i, k_d = k_d, sum_limit = sum_limit,
        truck_id = truck_id)
    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    controller.run()



if __name__ == '__main__':
    main(sys.argv)
