#!/usr/bin/env python

# Currently errors in model, incorrect code, or in need of tuning.
# Class for running MPC path following with one truck. Uses Frenet states.
# Uses topic to publish data to trucks via datasender, instead of trucksender.

import rospy

import numpy as np
import scipy.sparse as sparse

import math
import sys

from platoon.msg import truckmocap
from platoon.msg import truckcontrol
import path
import mpcsolver
import translator

class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, address, node_name, mocap_topic_type, mocap_topic_name,
        truck_topic_type, truck_topic_name, Ts,
        xyr = [1, 1], xyc = [0, 0], v_ref = 1, truck_id = 2):

        self.truck_id = truck_id

        self.Ts = Ts        # Sampling time.
        self.v_ref = v_ref  # Reference speed in m/s.

        # Inputs
        self.ua = 0.    # Wheel angle pwm.

        # States
        self.y = 0.     # Distance from path.
        self.theta = 0. # Angle difference between truck and path angle.
        self.a = 0.     # Wheel angle.

        self.nu = 1     # Number of inputs.
        self.nx = 3     # Number of states.

        # Constraints
        self.umin = np.array([-400])
        self.umax = np.array([400])
        self.xmin = np.array([-2., -np.inf, -math.pi/6])
        self.xmax = np.array([2., np.inf, math.pi/6])

        # Costs
        self.Q_values = np.array([1.0, 0.0, 0.0])
        self.Q = sparse.diags(self.Q_values)
        self.QN = self.Q
        self.R_values = [0.1]
        self.R = sparse.diags(self.R_values)

        # Initial value
        self.x0 = np.zeros(3)

        # Reference values
        self.xref = np.array([0, 0, 0])

        # Horizon length
        self.N = 5

        # Values for dynamics
        self.l = 0.27                       # Length between wheel pairs.
        self.k2 = 10.0                        # 1 over alpha time constant.
        self.ku = math.pi/(6*400)*self.k2   # ua constant in alpha dynamics.

        # PWM values for the input signals
        self.angle_pwm = 1500
        self.speed_pwm = 1500

        # Create reference path
        self.pt = path.Path()
        self.set_reference_path(xyr, xyc)

        self.k = 0  # Iteration

        self.xsim = np.array([0.0, 0.0, 0.0])    # Simulated state
        self.xsim1 = self.xsim                        # Previous simulated state
        self.xsim2 = self.xsim

        # Setup MPC controller
        Ad, Bd = self._get_AB(0, 0)

        self.mpc = mpcsolver.MPCSolver(Ad, Bd, self.Q, self.QN, self.R, self.N,
            self.x0, self.umin, self.umax, self.xmin, self.xmax, self.xref)

        self.mpc.prob.verbose = False

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = ['v_ref', 'Q_y', 'Q_theta', 'Q_alpha', 'R_alpha',
            'N']

        # Setup.
        self.running = False    # Controlling if controller is running or not.

        # Setup subscriber node and publisher..
        rospy.init_node(node_name, anonymous = True)
        rospy.Subscriber(mocap_topic_name, mocap_topic_type, self._callback)
        self.pub = rospy.Publisher(truck_topic_name, truck_topic_type,
            queue_size = 1)

        self.translator = translator.Translator()

        print('\nMPC Frenet controller initialized. Truck {}.\n'.format(
            self.truck_id))


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

        if self.running:
            self._control(x, y, yaw, vel)


    def _control(self, x, y, yaw, vel):
        """Perform control actions from received data. Sends new values to
        truck. """
        index, _ = self.pt.get_closest([x, y]) # Closest point on path.

        sim = False

        # Get the current states. If simulated get the simulated state.
        if sim:
            self.y = self.xsim[0]
            self.theta = self.xsim[1]
            self.a = self.xsim[2]
        else:
            self.y = self.pt.get_ey([x, y])             # Distance from path.
            self.theta = yaw - self.pt.get_gamma(index) # Path angle deviation.
            self.a = self.ua*math.pi/(6*400)            # Wheel angle (approx.).


        # Get new linearized dynamics.
        Ad, Bd = self._get_AB(index, vel)
        self.mpc.updateDynamics(Ad, Bd)

        # Update initial state for MPC (always 0 since linearizing).
        self.x0 = np.zeros(3)
        self.mpc.updatex0(self.x0)

        current_u = np.array([self.ua])
        current_x = np.array([self.y, self.theta, self.a])

        # Update the maximum and minimum values for the linearized system.
        self.mpc.update_bounds(
            self.umin - current_u,
            self.umax - current_u,
            self.xmin - current_x,
            self.xmax - current_x)

        # Update the reference for the linearized system.
        self.mpc.update_q(self.xref - current_x)

        control = self.mpc.solveMpc()   # Solve MPC problem.

        print('{:4d} |'.format(self.k)),

        # Apply inputs and print if controller solved problem.
        if control[0] is not None:
            d_ua = control[0]

            # Control inputs.
            self.ua = self.ua + d_ua

            # Update simulated system (probably incorrect code).
            self.xsim = self.xsim + Ad.dot(self.xsim - self.xsim2) + \
                Bd.dot(control)
            self.xsim2 = self.xsim1
            self.xsim1 = self.xsim

            print('da: {:7.2f} ||'.format(
                control[0])),

        # If the controller did not solve the problem:
        else:
            print('             {} ||'.format(control)),

        # Set control values within the bounds. For sending to truck and to
        # make simulation correct.
        if self.ua < -400:
            self.ua = -400
        if self.ua > 400:
            self.ua = 400

        # Transform control values to the pwm signals sent to the truck.
        self.speed_pwm = self.translator.get_speed(self.v_ref)
        self.angle_pwm = 1500 + self.ua

        print('angle: {:4.0f}, speed: {:4.0f} ||'.format(
            self.angle_pwm, self.speed_pwm)),

        print('y: {:5.2f}'.format(self.y))

        # Send values to the truck.
        #self.sender.send_data(self.speed_pwm, self.angle_pwm)
        if self.running:
            self.pub.publish(self.truck_id, self.speed_pwm, self.angle_pwm)

        self.k += 1


    def sign(self, x):
        if x < 0:
            return -1
        else:
            return 1


    def _get_AB(self, index, vel):
        """Return discrete A and B matrices. Linearized and discretized
        system dynamics. """
        cc = self.pt.get_gammap(index)  # Path curvature at the closest point.

        # Calculate entries for the linearized A matrix.
        a11 = 0
        a12 = vel*math.cos(self.theta + self.a)*self.sign(-self.y)
        a13 = vel*math.cos(self.theta + self.a)*self.sign(-self.y)

        a21 = -vel*math.cos(self.theta + self.a)*(cc/(1 - cc*self.y))**2
        a22 = vel*math.sin(self.theta + self.a)*cc/(1 - cc*self.y)
        a23 = vel*(1/self.l*math.cos(2*self.a) + \
            math.sin(self.theta + self.a)*cc/(1 - cc*self.y))

        a31 = 0
        a32 = 0
        a33 = -self.k2


        # Continuous A matrix.
        Ac = sparse.csc_matrix([
            [a11, a12, a13],
            [a21, a22, a23],
            [a31, a32, a33]
        ])

        # Calculate entries for the linearized continuous B matrix.
        b11 = 0
        b21 = 0
        b31 = self.ku

        # Continuous B matrix.
        Bc = sparse.csc_matrix([
            [b11],
            [b21],
            [b31]
        ])

        # Discretize the continuous system using forward Euler method.
        Ad = Ac.multiply(self.Ts) + sparse.eye(self.nx)
        Bd = Bc.multiply(self.Ts)

        return Ad, Bd


    def set_adjustables(self, values):
        """Used by the GUI to set the adjustable values. values is a list with
        the same size as the list returned by get_adjustables(). The values
        should here be treated in the same order as specified in that list. """
        try:
            v_ref = float(values[0])
            Qy = float(values[1])
            Qt = float(values[2])
            Qa = float(values[3])
            R = float(values[4])
            N = int(values[5])
        except:
            print('Invalid values entered.')
            return

        self.v_ref = v_ref
        self.Q_values = np.array([Qy, Qt, Qa])
        self.R_values = np.array([R])
        self.N = N

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        return self.adjustables, [self.v_ref, self.Q_values[0],
            self.Q_values[1], self.Q_values[2], self.R_values[0], self.N]


    def stop(self):
        """Stops/pauses the controller. """
        self.pub.publish(self.truck_id, 1500, self.angle_pwm)
        self.pub.publish(self.truck_id, 1500, self.angle_pwm)
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


    def set_reference_path(self, radius, center = [0, 0], pts = 800):
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

        self.pt.gen_circle_path(
            [self.xr, self.yr], pts, [self.xc, self.yc])


    def run(self):
        """Runs the controller. Needs to be called if not using the GUI. """
        self.start()
        rospy.spin()
        self.stop()



def main():
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
    mocap_topic_name = 'truck_topic'
    mocap_topic_type = truckmocap

    truck_topic_name = 'truck_control'
    truck_topic_type = truckcontrol

    # Data for controller reference path.
    x_radius = 1.7
    y_radius = 1.2
    center = [0.3, -1.3]

    Ts = 0.05

    # Initialize controller.
    controller = Controller(address, node_name, mocap_topic_type,
        mocap_topic_name, truck_topic_type, truck_topic_name, Ts = Ts,
        xyr = [x_radius, y_radius], xyc = center, truck_id = truck_id)

    controller.run()


if __name__ == '__main__':
    main()
