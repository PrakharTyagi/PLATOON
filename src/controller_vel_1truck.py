#!/usr/bin/env python

import rospy
import math
import sys

from platoon.msg import truckmocap
import path
import trucksender
import translator

class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, address, node_name, topic_type, topic_name,
        v = 0, k_p = 0, k_i = 0, k_d = 0,
        k_pv = 0, k_iv = 0, k_dv = 0,
        e_ref = 0.5, distance_offset = 0.4, pwm_min = 1400):

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = ['k_p', 'k_i', 'k_d',
            'k_pv', 'k_iv', 'k_dv']

        # Velocity of the truck and PID parameters.
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

        self.k_pv = k_pv
        self.k_iv = k_iv
        self.k_dv = k_dv

        self.sumy = 0               # Accumulated error.

        self.sum_e = 0

        self.old_e_rel = 0  # Keep track of previous error value.

        self.e_ref = e_ref
        self.distance_offset = distance_offset
        self.pwm_offset = 1450
        self.pwm_min = pwm_min
        self.pwm_max = 1500

        # Radii and center for reference path ellipse.
        self.xr = 0
        self.yr = 0
        self.xc = 0
        self.yc = 0

        self.stop_angle = 1500  # Angle used when stopping the truck.

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

        print('\nController_vel_1truck initialized.\n')


    def _callback(self, data):
        """Called when the subscriber receives data. """
        x1 = data.x1
        y1 = data.y1
        yaw1 = data.yaw1
        vel1 = data.velocity1

        x2 = data.x2
        y2 = data.y2
        yaw2 = data.yaw2
        vel2 = data.velocity2

        timestamp = data.timestamp

        self._control(x1, y1, yaw1, vel1, x2, y2, yaw2, vel2)


    def _control(self, x1, y1, yaw1, vel1, x2, y2, yaw2, vel2):
        """Perform control actions from received data. Sends new values to
        truck. """
        if self.running:

            omega = self._get_omega(x2, y2, yaw2, vel2)
            angle = int(self.translator.get_angle(omega, vel2))

            vel = self._get_velocity(x1, y1, vel1, x2, y2, vel2)

            vel_new = self.pwm_offset - vel

            if vel_new < self.pwm_min:
                vel_new = self.pwm_min
            if vel_new > self.pwm_max:
                vel_new = self.pwm_max

            print('pwm {:4.0f}'.format(vel_new))

            self.sender.send_data(vel_new, angle)

            self.stop_angle = angle


    def _get_omega(self, x, y, yaw, vel):
        """Calculate the control input omega. """

        index, closest = self.pt.get_closest([x, y]) # Closest point on path.

        ey = self.pt.get_ey([x, y])     # y error (distance from path)

        self.sumy = self.sumy + ey      # Accumulated error.

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

        #print(
        #    'Ctrl error: {:5.2f},  sum error: {:7.2f},  omega: {:5.2f}'.format(
        #    ey, self.sumy, omega))

        return omega

    def _get_velocity(self, x1, y1, vel1, x2, y2, vel2):
        e_dist = self.pt.get_distance([x1, y1],[x2, y2])
        try:
            e_time = (e_dist - self.distance_offset) / vel2
        except Exception as e:
            e_time = e_dist

        e_rel = self.e_ref - e_time

        e_p = e_rel - self.old_e_rel

        self.old_e_rel = e_rel

        k_p = self.k_pv
        k_i = self.k_iv
        k_d = self.k_dv

        self.sum_e = self.sum_e + e_rel     # Accumulated error.

        # PID controller.
        u = - k_p*e_rel - k_d*e_p - k_i * self.sum_e

        vel = u

        print(
           'Ctrl error: {:5.2f},  v2: {:7.2f},  vel: {:5.2f}, d = {:5.2f}'.format(
           e_rel, vel2, vel, e_dist)),

        return vel

    def _sign(self, x):
        """Returns the sign of x. """
        if x > 0:
            return 1
        else:
            return -1


    def stop(self):
        """Stops/pauses the controller. """
        self.sender.stop_truck(self.stop_angle)
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

            k_pv = float(values[3])
            k_iv = float(values[4])
            k_dv = float(values[5])

        except:
            print('\nInvalid control parameters entered.')
            return

        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.sumy = 0

        self.k_pv = k_pv
        self.k_iv = k_iv
        self.k_dv = k_dv
        self.sum_e = 0

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        return self.adjustables, [self.k_p, self.k_i, self.k_d,
            self.k_pv, self.k_iv, self.k_dv]


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


def main(args):
    address = ('192.168.1.193', 2390)   # Truck 1 address. (orange)
    try:
        if int(args[1]) == 1:
                address = ('192.168.1.194', 2390)   # Truck address.
    except:
        pass

    # Information for controller subscriber.
    node_name = 'controller_sub'
    topic_name = 'truck_topic'
    topic_type = truckmocap

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0.3, -1.3]

    # Controller tuning variables.
    v = 0.89             # Velocity used by translator model.

    k_p = 0.5
    k_i = -0.02
    k_d = 3

    k_pv = 10
    k_iv = 1
    k_dv = 5
    e_ref = 0.5
    distance_offset = 0.4

    # Initialize controller.
    controller = Controller(address, node_name, topic_type, topic_name,
        v = v, k_p = k_p, k_i = k_i, k_d = k_d,
        k_pv = k_pv, k_iv = k_iv, k_dv = k_dv,
        e_ref = e_ref, distance_offset = distance_offset)
    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    controller.run()



if __name__ == '__main__':
    main(sys.argv)
