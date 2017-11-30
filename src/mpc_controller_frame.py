#!/usr/bin/env python

#import rospy

#from platoon.msg import truckmocap
import path
import trucksender
import translator
import time
import scipy.optimize as opt
import numpy as np


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
        """Perform control actions from received data. Sends new values to
        truck. """
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
        self.nz = 4  # Number of states
        self.nu = 2  # Number of inputs
        self.Qz =   np.array([
                    [0, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 0]
                                ])  # Inital Qz matrix
        self.Qu = np.array([
                    [1, 0],
                    [0, 1]
                                ]) # Initial Qu matrix
        #self.Qf = Qf  # Final Qf matrix
        self.N = 5  # Control horizion
        self.Ts = 1 / 20  # Samling time
        #self.margin = margin  # margIIIn
       # self.constraints = (  # Initial constraints struct
       #     {
        #        'type': 'eq',
        #        'fun': self.equality_constraints
         #   },
         #   {
         #       'type': 'ineq',
         #       'fun': self.inequality_constraints
          #  }
        #)

    def xTQx(self, x, Q):
        '''Returns the value of x^TQx'''
        return np.dot(np.dot(np.transpose(x), Q), x)

    def get_control(self, x, i):
        '''Gettter of the control vector at time instance i from the optimization
           variable x.'''

    def get_state(self, x, i):
        '''Gettter of the state vector at time instance i from the optimization
           variable x.'''

    def set_state(self, x, z, i):
        '''Setter for a state vector z at time instance i for the optimization
           variable x.'''



    def calc_control(self, z):
        '''Calculate the MPC control given the current state z and the previous
           wheel angle beta. Solves the optimization problem and returns the
           first input step, the evaluated cost, the number of iterations performed
           by the solved and the total solve time.'''
        self.z0 = z
        t_start = time.process_time()
        init_guess = np.array([0,0,0,0])#self.calc_init_guess()

        res = opt.minimize(self.cost_function,
                           init_guess,
                           method='SLSQP',
                           constraints=self.constraints,
                           bounds=self.bounds,
                           options={
                               'maxiter': 150,
                               'disp': True,
                               'ftol': 0.01
                          })
        t_solve = time.process_time() - t_start
        #if partial_tracking:
         #   self.print_partial_progress(res.x, init_guess)
        #self._x_opt = res.x
        u_opt = self.get_control(res.x, 0)
        #self.beta0 = self.get_beta(u_opt)
        return u_opt, res.fun, res.nit, t_solve


    #def get_state(self, x, i):
      #  '''Gettter of the state vector at time instance i from the optimization
      #     variable x.'''
        #if i == 0:
        #    return self.z0
        #start = self.nz*(i-1)
        #return x[start: start + self.nz]

    def getPos(self, x, k):
        '''Calculates the reference error \hat{r} - r_{ref}'''
        pos = self.get_state(x, k)
        if k == 0:
            return self.z0
        start = self.nz*(k-1)
        return x[start: start + self.nz]

    def cost_function(self, x):
        J = 0
        for k in range(self.N-1):
            pos = self.getPos(x,k)
            yPos = x(2)

            #ref_cost = self.xTQx(ref_err, self.Qz)
            #curr_u = self.get_control(x, k)
            #u_cost = self.xTQx(curr_u, self.Qu)
            #J = J + ref_cost + u_cost
        #ref_err = self.get_ref_err(x, self.reference, k)
        #term_cost = self.xTQx(ref_err, self.Qf)
        #return J + term_cost

   # def equality_constraints(self, x):

    # return self.get_model_constraints(x)

   # def inequality_constraints(self, x):
    # return self.get_input_constraints(x)






