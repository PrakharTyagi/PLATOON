#!/usr/bin/env python

#import rospy

#from platoon.msg import truckmocap
import path
import trucksender
import translator
import time
import scipy.optimize as opt
import numpy as np

import osqp
import scipy as sp
import scipy.sparse as sparse


class Controller():
    """Class for subscribing to topic mocap data, calculate control input and
    send commands to the truck. """
    def __init__(self, address, node_name, topic_type, topic_name):

        # List of strings used by the GUI to see which values it can adjust.
        self.adjustables = []
        # Information for subscriber node.

        self.mpc = self.mpcDynaics()
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

        print('\nController initialized.\n')


    def _callback(self, data):
        """Called when the subscriber receives data. """
        x = data.x2
        y = data.y2
        yaw = data.yaw2
        timestamp = data.timestamp
        vel = data.velocity2

        self._control(x, y, yaw, vel)


    def mpcDynaics(self):
        Ad = sparse.csc_matrix([
            [1., 0., 0.],
            [0., 1., 0.],
            [0., 0., 1]
        ])
        Bd = sparse.csc_matrix([
            [0.5, .0],
            [0.5, 0.],
            [0., 0.5]
        ])

        Q = sparse.diags([0.1, 0.1, 0.1])
        QN = Q
        R = 0.1 * sparse.eye(2)
        umin = np.array([0, 0])
        umax = np.array([1, 1])
        xmin = np.array([-10, -10, -10])
        xmax = np.array([10, 10, 10])
        x0 = np.zeros(3)

        N = 3

        xref = np.array([6, 6, 6])
        mpc = Mpc_controller(Ad, Bd, Q, QN, R, N, x0, umin, umax, xmin, xmax, xref)
        return mpc

    def _control(self, x, y, yaw, vel):
        self.mpc.updatex0([x,y,yaw])
        ctrl = self.mpc.solveMpc()
        return ctrl


    def set_adjustables(self, values):
        """Used by the GUI to set the adjustable values. values is a list with
        the same size as the list returned by get_adjustables(). The values
        should here be treated in the same order as specified in that list. """

        print('\nControl parameter changes applied.')


    def get_adjustables(self):
        """Used by the GUI to get the values that are adjustable.
        Returns two lists. The first list is a list of the names/descriptors
        of the adjustable parameters. The second is the current values of those
        parameters. """
        return self.adjustables, []


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
    topic_name = 'truck_topic'
    topic_type = truckmocap

    # Data for controller reference path.
    x_radius = 1.6
    y_radius = 1.2
    center = [0.3, -0.5]

    # Initialize controller.
    controller = Controller(address, node_name, topic_type, topic_name)
    # Set reference path.
    controller.set_reference_path([x_radius, y_radius], center)

    controller.run()


if __name__ == '__main__':
    main()


class Mpc_controller:

    def __init__(self,Ad,Bd,Q,QN,R,N,x0,umin,umax,xmin,xmax,xr):

        self.Ad = Ad
        self.Bd = Bd
        [self.nx, self.nu] = Bd.shape

        self.Q  = Q
        self.QN = QN
        self.R  = R
        self.N = N
        self.x0 = x0
        self.umin = umin
        self.umax = umax
        self.xmin = xmin
        self.xmax = xmax
        self.xr = xr
        self.prob = osqp.OSQP()
        self.updateController()


    def updateDynamics(self,Ad,Bd):
        self.Ad = Ad
        self.Bd = Bd
        [self.nx, self.nu] = Bd.shape
        self.updateController()

    def updateMaxMin(self,umin,umax,xmin,xmax):
        self.umin = umin
        self.umax = umax
        self.xmin = xmax
        self.xmax = xmax
        self.updateController()

    def updateRef(self,xr):
        self.xr = xr
        self.updateController()
    def updateHorizon(self,N):
        self.N = N
        self.updateController()
    def updatex0(self,x0):
        self.x0 = x0
        self.updateController()
    def updateObjective(self,Q,QN,R):
        self.Q  = Q
        self.QN = QN
        self.R  = R
        self.updateController()

    def updateController(self):
        P = sparse.block_diag([sparse.kron(sparse.eye(self.N), self.Q), self.QN,
                               sparse.kron(sparse.eye(self.N), self.R)])
        # - linear objective
        q = np.hstack([np.kron(np.ones(self.N), -self.Q.dot(self.xr)), -self.QN.dot(self.xr),
                       np.zeros(self.N * self.nu)])

        Ax = sparse.kron(sparse.eye(self.N + 1), -sparse.eye(self.nx)) + sparse.kron(sparse.eye(self.N + 1, k=-1), self.Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), self.Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-self.x0, np.zeros(self.N * self.nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)
        lineq = np.hstack([np.kron(np.ones(self.N + 1), self.xmin), np.kron(np.ones(self.N), self.umin)])
        uineq = np.hstack([np.kron(np.ones(self.N + 1), self.xmax), np.kron(np.ones(self.N), self.umax)])
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq])
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])
        self.prob.setup(P, q, A, l, u, warm_start=True)

    def solveMpc(self):
        res = self.prob.solve()
        ctrl = res.x[-self.N * self.nu:-(self.N - 1) * self.nu]

        return ctrl
