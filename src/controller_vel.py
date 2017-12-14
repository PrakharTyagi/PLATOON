#!/usr/bin/env python

import rospy
import math
import sys
import time

from platoon.msg import truckmocap
from platoon.msg import truckcontrol
import path
import translator
import frenetpid

class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self,
        node_name, mocap_topic_type, mocap_topic_name,
        truck_topic_type, truck_topic_name,
        v = 0, k_p1 = 0, k_i1 = 0, k_d1 = 0,
        k_p2 = 0, k_i2 = 0, k_d2 = 0,
        k_pv = 0, k_iv = 0, k_dv = 0,
        e_ref = 0.5, distance_offset = 0.4, pwm_min = 1400, pwm_max = 1460,
        follower = 2, vlim = 0.5):

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = ['v_lead',
            'k_p1', 'k_i1', 'k_d1',
            'k_p2', 'k_i2', 'k_d2',
            'k_pv', 'k_iv', 'k_dv',
            'e_ref']

        self.follower = follower    # The truck (1 or 2) that is follower.

        # Velocity controller PID parameters.
        self.k_pv = k_pv
        self.k_iv = k_iv
        self.k_dv = k_dv

        self.sum_e = 0

        self.v_lead = v             # The desired speed of the leader truck.

        self.e_ref = e_ref          #
        self.distance_offset = distance_offset # Compensate for truck lengths.
        self.pwm_min = pwm_min      # Minimum safe pwm.
        self.pwm_max = pwm_max      # 1500 max to prevent backwards driving.

        self.old_e_rel = 0          # Used for derivative of control error.

        self.vlim = vlim            # Min lead speed for which follower acts.

        # Radii and center for reference path ellipse.
        self.xr = 0
        self.yr = 0
        self.xc = 0
        self.yc = 0

        # Angles used when sending the stop signal to the trucks.
        self.stop_angle1 = 1500
        self.stop_angle2 = 1500

        self.running = False    # Controlling if controller is running or not.

        # Setup subscriber node.
        rospy.init_node(node_name, anonymous = True)
        rospy.Subscriber(mocap_topic_name, mocap_topic_type, self._callback)
        self.pub = rospy.Publisher(truck_topic_name, truck_topic_type,
            queue_size = 1)

        # Create reference path object, translator, and sender.
        self.pt = path.Path()
        self.translator = translator.Translator()

        # Create Frenet PID controllers for path following for both trucks.
        self.frenet1 = frenetpid.FrenetPID(self.pt, k_p1, k_i1, k_d1)
        self.frenet2 = frenetpid.FrenetPID(self.pt, k_p2, k_i2, k_d2)

        print('\nController vel initialized. Truck {} is follower. \n'.format(
            self.follower
        ))


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
        if not self.running:
            return

        omega1 = self.frenet1.get_omega(x1, y1, yaw1, vel1)
        angle1 = int(self.translator.get_angle(omega1, vel1))

        omega2 = self.frenet2.get_omega(x2, y2, yaw2, vel2)
        angle2 = int(self.translator.get_angle(omega2, vel2))

        v_lead_pwm = int(self.translator.get_speed(self.v_lead))

        if self.follower == 2:
            v1_pwm = v_lead_pwm
            v1_pwm = self._bound_pwm(v1_pwm)

            if vel1 < self.vlim:
                v2_pwm = 1500
            else:
                v2_pwm = v1_pwm - self._get_velocity(x1, y1, vel1, x2, y2, vel2)
                v2_pwm = self._bound_pwm(v2_pwm)

        else:
            v2_pwm = v_lead_pwm

            if vel2 < self.vlim:
                v1_pwm = 1500
            else:
                v1_pwm = v2_pwm - self._get_velocity(x2, y2, vel2, x1, y1, vel1)
                v2_pwm = self._bound_pwm(v2_pwm)


        print('pwm1: {:.0f}, pwm2: {:.0f}'.format(v1_pwm, v2_pwm))

        self.pub.publish(1, v1_pwm, angle1)
        self.pub.publish(2, v2_pwm, angle2)

        self.stop_angle1 = angle1
        self.stop_angle2 = angle2


    def _bound_pwm(self, pwm):
        """Returns a pwm signal within the minimum and maximum values. """
        if pwm < self.pwm_min:
            pwm = self.pwm_min
            
        if pwm > self.pwm_max:
            pwm = self.pwm_max

        return pwm


    def _get_velocity(self, x1, y1, vel1, x2, y2, vel2):
        e_dist = self.pt.get_distance([x1, y1], [x2, y2])
        try:
            e_time = (e_dist - self.distance_offset) / vel2
        except:
            e_time = e_dist - self.distance_offset

        e_rel = self.e_ref - e_time

        e_p = e_rel - self.old_e_rel
        self.old_e_rel = e_rel

        k_p = self.k_pv
        k_i = self.k_iv
        k_d = self.k_dv

        self.sum_e = self.sum_e + e_rel     # Accumulated error.

        # PID controller.
        u = - k_p*e_rel - k_d*e_p - k_i * self.sum_e
        if e_rel > 0:
            u = u - 10*k_p*e_rel

        vel = u

        print(
           'Ctrl e: {:5.2f},  u: {:7.2f},  v1: {:5.2f}, v2: {:5.2f},'.format(
           e_rel, u, vel1, vel2)),

        return vel


    def _sign(self, x):
        """Returns the sign of x. """
        if x > 0:
            return 1
        else:
            return -1


    def stop(self):
        """Stops/pauses the trucks and the controller. """
        t = 0.05

        self.pub.publish(1, 1500, self.stop_angle1)
        time.sleep(t)
        self.pub.publish(2, 1500, self.stop_angle2)
        time.sleep(t)

        self.pub.publish(2, 1500, self.stop_angle2)
        time.sleep(t)
        self.pub.publish(1, 1500, self.stop_angle1)
        time.sleep(t)

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
            v_lead = float(values[0])

            k_p1 = float(values[1])
            k_i1 = float(values[2])
            k_d1 = float(values[3])

            k_p2 = float(values[4])
            k_i2 = float(values[5])
            k_d2 = float(values[6])

            k_pv = float(values[7])
            k_iv = float(values[8])
            k_dv = float(values[9])
            e_ref = float(values[10])

        except:
            print('\nInvalid control parameters entered.')
            return

        self.v_lead = v_lead

        self.frenet1.set_pid(k_p1, k_i1, k_d1)

        self.frenet2.set_pid(k_p2, k_i2, k_d2)

        self.k_pv = k_pv
        self.k_iv = k_iv
        self.k_dv = k_dv

        self.sum_e = 0

        self.e_ref = e_ref

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        k_p1, k_i1, k_d1 = self.frenet1.get_pid()
        k_p2, k_i2, k_d2 = self.frenet2.get_pid()

        return self.adjustables, [self.v_lead,
            k_p1, k_i1, k_d1,
            k_p2, k_i2, k_d2,
            self.k_pv, self.k_iv, self.k_dv,
            self.e_ref]


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
    # Choose which truck is the follower truck. 2 if 1 is not entered as arg.
    follower = 2
    try:
        if int(args[1]) == 1:
            follower = 1
    except:
        pass

    # Information for controller subscriber.
    node_name = 'controller_sub'
    mocap_topic_name = 'truck_topic'
    mocap_topic_type = truckmocap

    truck_topic_name = 'truck_control'
    truck_topic_type = truckcontrol

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0.3, -1.3]

    # Controller tuning variables.
    v = 0.89

    k_p1 = 0.5
    k_i1 = -0.02
    k_d1 = 3

    k_p2 = 0.5
    k_i2 = -0.02
    k_d2 = 3

    k_pv = 10
    k_iv = 1
    k_dv = 5
    e_ref = 0.5
    distance_offset = 0.4

    # Initialize controller.
    controller = Controller(
        node_name, mocap_topic_type, mocap_topic_name,
        truck_topic_type, truck_topic_name,
        v = v, k_p1 = k_p1, k_i1 = k_i1, k_d1 = k_d1,
        k_p2 = k_p2, k_i2 = k_i2, k_d2 = k_d2,
        k_pv = k_pv, k_iv = k_iv, k_dv = k_dv,
        e_ref = e_ref, distance_offset = distance_offset,
        follower = follower)
    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    print('Recommended to use this controller in the GUI because of problems '\
        'with stopping the trucks after termination of the script.')
    #controller.run()

if __name__ == '__main__':
    main(sys.argv)
