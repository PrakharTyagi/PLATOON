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

        self.mpc = Mpc_controller()
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
        x = data.x
        y = data.y
        yaw = data.yaw
        timestamp = data.timestamp
        vel = data.velocity

        self._control(x, y, yaw, vel)


    def _control(self, x, y, yaw, vel):
        self.mpc.solveMpc()
        self.sender.send_data(1500, 1500)


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
    topic_name = 'truck2'
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

    def __init__(self):
        #self.u0 = 0.
        #self.x0 = np.zeros(2)

        self.N = 10    #Init Control Horizon
        self.h = 1/20  #Init Sample Frequency
        self.Ad, self.Bd, self.nx, self.nu = self.getInitVehicleDynamics()
        self.Q, self.QN, self.R = self.getInitObjective()
        self.umin, self.umax, self.xmin, self.xmax = self.getInitMaxMin()
        self.Aineq, self.lineq, self.uineq = self.ineqConstraints(self.N, self.nu, self.nx,self.umin, self.umax, self.xmin, self.xmax)
        self.x0 = np.zeros(3)
        #Updated every seq (because of x0.)
        #Aineq, lineq, uineq = self.ineqConstraints(self.N,nu,nx,xmin,xmax,umin,umax) #Updated every..
        # Create an OSQP object
        self.prob = osqp.OSQP()
        #self.setupWorkspace()

    #Update dynamics by typing in new Ad and Bd
    def updateDynamics(self,Ad,Bd):
        self.Ad = Ad
        self.Bd = Bd
        [self.nx, self.nu] = self.Bd.shape
    #Update Control horizon
    def updateHorizon(self,N):
        self.N = N

    def updateSampleFreq(self,h):
        self.h = h

    def updateCost(self,Q,QN,R):
        self.Q = Q
        self.QN = QN
        self.R = R

    def updateMaxMin(self,umin,umax,xmin,xmax):
        self.umin = umin
        self.umax = umax
        self.xmin = xmin
        self.xmax = xmax

    #Linearize every sequence in order to get a proper control.
    def linearizeInSequence(self,yaw):
        self.Bd = sparse.csc_matrix([
            [self.h*np.cos(yaw-np.pi/2),0],
            [self.h*np.sin(yaw-np.pi/2),0],
            [0,self.h]
        ])
        [self.nx, self.nu] = self.Bd.shape


    #Generate a path on a circle based on this.
    def generatePath(self,x,y,r):
        theta = np.arctan2(float(y), float(x))
        Larc = 0.3
        newTheta = Larc/r
        thetaR = theta+newTheta
        xNew = r*np.cos(thetaR)
        yNew = r*np.sin(thetaR)
        return xNew, yNew,(thetaR+np.pi/2)




    def solveSimpleMpc(self,x,y,yaw,xref):
        self.x0 =[x,y,yaw]
        xr = xref
        yr = y
        thetar = yaw
        #self.linearizeInSequence(yaw)
        Aeq, leq, ueq = self.linearDynamics(self.Ad, self.Bd, self.nx, self.x0, self.N)
        P, q = self.castMpc(self.nu, self.Q, self.QN, self.R, [xr, yr, thetar])  # into solver!
        A,l,u = self.getOSPConstraints(Aeq,self.Aineq,leq,self.lineq,ueq,self.uineq)

        self.prob.setup(P, q, A, l, u, warm_start=True)
            # Solve
        res = self.prob.solve()
            # Check solver status
       # if res.info.status != 'Solved':
       #    raise ValueError('OSQP did not solve the problem!')
    # Apply first control input to the plant
        return res.x[-self.N * self.nu:-(self.N - 1) * self.nu]





    def solveMpc(self,x,y,r,yaw):
        self.x0 =[x,y,yaw]
        #xr,yr,thetar = self.generatePath(x,y,r)
        self.linearizeInSequence(yaw)
        Aeq, leq, ueq = self.linearDynamics(self.Ad, self.Bd, self.nx, self.x0, self.N)
        P, q = self.castMpc(self.nu, self.Q, self.QN, self.R, [xr, yr, thetar])  # into solver!
        A,l,u = self.getOSPConstraints(Aeq,self.Aineq,leq,self.lineq,ueq,self.uineq)

        self.prob.setup(P, q, A, l, u, warm_start=True)
            # Solve
        res = self.prob.solve()
            # Check solver status
       # if res.info.status != 'Solved':
       #    raise ValueError('OSQP did not solve the problem!')
    # Apply first control input to the plant
        return res.x[-self.N * self.nu:-(self.N - 1) * self.nu]

        #self.x0 = self.Ad.dot(self.x0) + self.Bd.dot(ctrl)
    #def setupWorkspace(self):
        #self.prob.setup(self.P, self.q, self.A, self.l, self.u, warm_start=True)


    def getNextState(self,ctrl):
        u1 = ctrl[0]
        u2 = ctrl[1]
        z = self.Ad*self.x0 +self.Bd*[u1, u2]
        return z


    def linearDynamics(self,Ad,Bd,nx,x0,N):
        #print(nx)
        Ax = sparse.kron(sparse.eye(self.N + 1), -sparse.eye(nx)) + sparse.kron(sparse.eye(N + 1, k=0), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([x0, np.zeros(N * nx)])
        ueq = leq
        return Aeq, leq, ueq

    def getOSPConstraints(self,Aeq,Aineq,leq,lineq,ueq,uineq):
        A = sparse.vstack([Aeq, Aineq])
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])
        return A,l,u


    def ineqConstraints(self,N,nu,nx,umin,umax,xmin,xmax):
        Aineq = sparse.eye((N + 1) * nx + N * nu)
        lineq = np.hstack([np.kron(np.ones(N + 1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N + 1), xmax), np.kron(np.ones(N), umax)])
        return Aineq, lineq, uineq

    def castMpc(self,nu,Q,QN,R,xr):

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        #Q,QN,R = self.getObjective()
        P = sparse.block_diag([sparse.kron(sparse.eye(self.N), Q), QN, sparse.kron(sparse.eye(self.N), R)])
        # - linear objective
        #q = np.hstack([np.kron(np.ones(self.N), -Q.dot(xr)), -QN.dot(xr),
        #               np.zeros(self.N * nu)])
        q = np.hstack([np.kron(np.ones(self.N), -Q.dot(xr)), -QN.dot(xr),
                       np.zeros(self.N * nu)])
        return P, q

    def getRef(self):
        xr = np.array([6., 6., 6.])
        return xr

    def getInitObjective(self):
        Q = sparse.diags([0.1, 0.1, 0.1])
        QN = Q
        R = 0.1 * sparse.eye(2)
        return Q,QN,R

    def getInitVehicleDynamics(self):
        Ad = sparse.csc_matrix([
                [1., 0., 0.],
                [0., 1., 0.],
                [0., 0., 1.]
            ])
        Bd = sparse.csc_matrix([
            [0, .0],
            [self.h, .0],
            [.0, self.h]
        ])
        [nx, nu] = Bd.shape
        return Ad, Bd, nx, nu


    def getInitMaxMin(self):
        umin = np.array([-1.,-1.])
        umax = np.array([1.,1.])
        xmin = np.array([-5, -5, -1*np.pi])
        xmax = np.array([5, 5, np.pi])
        return umin, umax, xmin, xmax

        # Use this badboy for testing
        def solveMpcTest(self, x, ref, Ad, Bd, QQ, QQN, RR, N, umin, umax, xmin, xmax):
            self.x0 = x
            self.N = N
            self.Ad = Ad
            self.Bd = Bd

            [self.nx, self.nu] = self.Bd.shape
            Aeq, leq, ueq = self.linearDynamics(self.Ad, self.Bd, self.nx, self.x0, self.N)
            P = sparse.block_diag([sparse.kron(sparse.eye(self.N), QQ), QQN, sparse.kron(sparse.eye(self.N), RR)])
            q = np.hstack([np.kron(np.ones(self.N), -QQ.dot(ref)), -QQN.dot(ref),
                           np.zeros(self.N * self.nu)])
            umin = umin
            umax = umax
            xmin = xmin
            xmax = xmax

            Aineq = sparse.eye((N + 1) * self.nx + self.N * self.nu)
            lineq = np.hstack([np.kron(np.ones(self.N + 1), xmin), np.kron(np.ones(self.N), umin)])
            uineq = np.hstack([np.kron(np.ones(self.N + 1), xmax), np.kron(np.ones(self.N), umax)])
            A = sparse.vstack([Aeq, Aineq])
            l = np.hstack([leq, lineq])
            u = np.hstack([ueq, uineq])

            self.prob.setup(P, q, A, l, u, warm_start=True)
            # Solve
            res = self.prob.solve()
            # Check solver status
            # if res.info.status != 'Solved':
            #    raise ValueError('OSQP did not solve the problem!')
            # Apply first control input to the plant
            ctrl = res.x[-self.N * self.nu:-(self.N - 1) * self.nu]
            x0 = Ad.dot(self.x0) + Bd.dot(ctrl)

            # Update initial state
            l[:self.nx] = -x0
            u[:self.nx] = -x0
            self.prob.update(l=l, u=u)

            return res.x[-self.N * self.nu:-(self.N - 1) * self.nu]


        # Setup workspace


