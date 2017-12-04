#!/usr/bin/env python

import rospy
import math

from platoon.msg import truckmocap
import path
import trucksender
import translator
import time

class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, address, node_name, topic_type, topic_name,
        v = 0, k_p = 0, k_i = 0, k_d = 0, sum_limit = 0):

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = ['k_p', 'k_i', 'k_d', 'v', 'sum_limit']

        # Velocity of the truck and PID parameters.
        self.v = v
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.sumy = 0               # Accumulated error.
        self.sum_limit = sum_limit  # Limit for accumulated error.

        self.I_term = 0             #accumulated error for distance controller
        self.current_time = time.time()
        self.last_time = self.current_time

        # Radii and center for reference path ellipse.
        self.xr = 0
        self.yr = 0
        self.xc = 0
        self.yc = 0

        # Information for subscriber node.
        self.node_name = node_name
        self.topic_name = topic_name
        self.topic_type = topic_type

        self.address = address  # IP-address of the truck to be controlled.

        self.running = False    # Controlling if controller is running or not.

        # Setup subscriber node.
        rospy.init_node(self.node_name, anonymous = True)
        rospy.Subscriber(self.topic_name, self.topic_type, self._callback)

        # Create reference path object, translator, and sender.
        self.pt = path.Path()
        self.translator = translator.Translator()
        self.sender = trucksender.TruckSender(self.address)

        adress2 = address
        self.sender2 = trucksender.TruckSender(adress2)

        self.v_pwm = self.translator.getSpeed() # PWM velocity.
        self.v_pwm2 = self.translator.getSpeed()

        print('\nController initialized.\n')


    def _callback(self, data):
        """Called when the subscriber receives data. """
        x = data.x1
        y = data.y1
        yaw = data.yaw1
        vel = data.velocity1

        x2 = data.x2
        y2 = data.y2
        yaw2 = data.yaw2
        vel2 = data.velocity2

        timestamp = data.timestamp
        self._control(x, y, yaw, vel, x2, y2, yaw2, vel2)


    def _control(self, x, y, yaw, vel, x2, y2, yaw2, vel2):
        """Perform control actions from received data. Sends new values to
        truck. """
        if self.running:

            omega = self._get_omega(x, y, yaw, vel)
            omega2 = self._get_omega(x2, y2, yaw2, vel2)

            angle = int(self.translator.get_angle(omega, vel))
            angle2 = int(self.translator.get_angle(omega2, vel2))

            self.v_pwm = self.translator.get_speed(self.v) # pwm value.
            self.v_pwm2 = self.translator.get_speed(self.v)

            self.sender.send_data(self.v_pwm, angle)
            self.sender.send_data(self.v_pwm2, angle2)

    def distance_controller (self, x1, y1, x2, y2):
        """Distance PID controller. Input: Distance to the first truck
                                Setpoint: 0.5 m
                                Output: Control to the motor"""

        set_point = 0.5
        dist_diff = sqrt(x1**2 + y1**2)-sqrt(x2**2 + y2**2)

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time

        e_dist = set_point-dist_diff

        kp_dist = 0
        ki_dist = 0
        kd_dist = 0

        self.P_term = kp_dist * e_dist
        self.I_term = self.I_term + e_dist*delta_time

        u_dist = self.P_term + ki_dist * self.I_term

    def _get_omega(self, x, y, yaw, vel):
        """Calculate the control input omega. """

        index, closest = self.pt.get_closest([x, y]) # Closest point on path.

        ey = self.pt.get_ey([x, y])     # y error (distance from path)

        self.sumy = self.sumy + ey      # Accumulated error.
        if self.sumy > self.sum_limit:
            self.sumy = self.sum_limit
        if self.sumy < -self.sum_limit:
            self.sumy = -self.sum_limit

        gamma = self.pt.get_gamma(index)
        gamma_p = self.pt.get_gammap(index)
        gamma_pp = self.pt.get_gammapp(index)

        cos_t = math.cos(yaw - gamma)     # cos(theta)
        sin_t = math.sin(yaw - gamma)     # sin(theta)

        # y prime (derivative w.r.t. path).
        yp = math.tan(yaw - gamma)*(1 - gamma_p*ey)*self._sign(
            vel*cos_t/(1 - gamma_p*ey))

        # PID controller.
        u = - self.k_p*ey - self.k_d*yp - self.k_i * self.sumy

        # Feedback linearization.
        omega = vel*cos_t/(1 - gamma_p*ey) * (u*cos_t**2/(1 - gamma_p*ey) +
                                gamma_p*(1 + sin_t**2) +
                                gamma_pp*ey*cos_t*sin_t/(1 - gamma_p*ey))

        print(
            'Ctrl error: {:5.2f},  sum error: {:7.2f},  omega: {:5.2f}'.format(
            ey, self.sumy, omega))

        return omega


    def _sign(self, x):
        """Returns the sign of x. """
        if x > 0:
            return 1
        else:
            return -1


    def stop(self):
        """Stops/pauses the controller. """
        self.sender.stop_truck()
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

        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.v = v
        self.v_pwm = self.translator.get_speed(self.v)
        self.sum_limit = sum_limit
        self.sumy = 0

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        return self.adjustables, [self.k_p, self.k_i, self.k_d, self.v,
            self.sum_limit]


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


    def run(self):
        """Runs the controller. Needs to be called if not using the GUI. """
        self.start()
        rospy.spin()
        self.stop()


def main():
    address = ('192.168.1.193', 2390)   # Truck address.

    # Information for controller subscriber.
    node_name = 'controller_sub'
    topic_name = 'truck2'
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
        v = v, k_p = k_p, k_i = k_i, k_d = k_d, sum_limit = sum_limit)
    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    controller.run()



if __name__ == '__main__':
    main()
