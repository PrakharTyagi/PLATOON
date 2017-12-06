#!/usr/bin/env python

import rospy
from platoon.msg import *
import time
import math
import sys
from mocap_source_2 import *

class MovingAverage:
    """Class for calculating a moving average. At start, the average of the so
    far entered numbers are returned. Insert a new value and get the updated
    moving average by calling new_ma(value). """
    def __init__(self, N):
        """N: how many numbers that should be averaged. """
        self.N = N

        self.values = [None for i in range(N)]  # List containing the values.
        self.i = 0      # Index keeping track of where to enter the new value.
        self.ma = 0     # The moving average.

    def new_ma(self, value):
        """Inserts the value and returns the new moving average.
        The oldest value in the list is replaced with the new value. """
        value = float(value)

        # If the list is not yet full (start of operation), add the new value
        # at the next position and update the moving average.
        if self.i < self.N:
            self.values[self.i] = value
            self.ma = self.ma*self.i/(self.i + 1) +  value/(self.i + 1)
            self.i += 1
        # If the list is full (normal operation) update the moving average and
        # replace the oldest value in the list with the new value.
        else:
            index = self.i % self.N
            self.ma = self.ma - self.values[index]/self.N + value/self.N
            self.values[index] = value
            self.i += 1

        return self.ma

    def get_ma(self):
        """Returns the moving average without inserting a new value. """
        return self.ma

class CircleTruck():
    """Class for simulating a truck driving in a circle. """
    def __init__(self, theta0 = 0):
        self.radius = 1.3
        self.theta0 = theta0     # Initial angle.
        self.velocity = 1

        self.theta = self.theta0            # Angle for position on circle.
        self.x = self.radius*math.cos(self.theta)
        self.y = self.radius*math.sin(self.theta)
        self.yaw = self.theta + math.pi/2   # Angle of truck.

    def update_pos(self, time_elapsed):
        """Update the position of the simulated truck. """
        self.theta = self.theta0 + time_elapsed*self.velocity/self.radius

        self.x = self.radius*math.cos(self.theta)
        self.y = self.radius*math.sin(self.theta)
        self.yaw = (self.theta + math.pi/2) % (2*math.pi)


    def get_values(self):
        """Return the position and orientation of the simulated truck. """
        return self.x, self.y, self.yaw


class Truck:
    """Class for getting data from Mocap. """
    def __init__(self,truck_name):
        self.mocap = Mocap(host = '192.168.1.10', info = 1)
        self.mocap_body = self.mocap.get_id_from_name(truck_name)

    def get_values(self):
        """Returns the truck state. """
        truck_state = self.mocap.get_body(self.mocap_body)
        x = truck_state['x']
        y = truck_state['y']
        yaw = truck_state['yaw']

        return x, y, yaw*math.pi/180


class TruckPublisher():
    """Publisher class. Keeps an instance of the mocap or simulation object. """
    def __init__(self, node_name, topic_type, topic_name,
                 update_freq = 20, mocap_used = True,
                 queue_size = 1, ma = 1):
        self.node_name = node_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.update_freq = update_freq
        self.mocap_used = mocap_used
        self.queue_size = queue_size

        circletruck_rad_distance = 0.4

        self.time_bw=0
        self.time_bw_prev=self.time_bw

        self.x1_pos=0
        self.x1_old=self.x1_pos
        self.y1_pos=0
        self.y1_old=self.y1_pos
        self.yaw1_pos=0
        self.yaw1_old=self.yaw1_pos

        self.x2_pos=0
        self.x2_old=self.x2_pos
        self.y2_pos=0
        self.y2_old=self.y2_pos
        self.yaw2_pos=0
        self.yaw2_old=self.yaw2_pos

        self.vma1 = MovingAverage(ma)
        self.vma2 = MovingAverage(ma)

        self.pub = rospy.Publisher(self.topic_name, self.topic_type,
            queue_size = self.queue_size)
        rospy.init_node(self.node_name, anonymous = True)

        self.rate = rospy.Rate(self.update_freq)
        self.time_elapsed = 0
        self.init_time = time.time()

        if self.mocap_used:
            self.tr1 = Truck("TruckVehicle1")
            self.tr2 = Truck("TruckVehicle2")
        else:
            self.tr1 = CircleTruck()
            self.tr2 = CircleTruck(circletruck_rad_distance)

        print('Publisher running')
        if not self.mocap_used:
            print('Using simulated trucks')


    def talker(self):
        """Publishes the mocap or simulated data continuously to the topic. """
        while not rospy.is_shutdown():

            self.time_elapsed = time.time() - self.init_time

            self.time_bw_prev=self.time_bw
            self.time_bw=(rospy.get_time())

            tid_mellan=(self.time_bw-self.time_bw_prev) #tid mellan datainhamtningar
            #print(tid_mellan)


            try:
                # If using simulation, update the position of the truck.
                if not self.mocap_used:
                    self.tr1.update_pos(self.time_elapsed)
                    self.tr2.update_pos(self.time_elapsed)
            except Exception as e:

                print "Unexpected error:", sys.exc_info()[0]
                print('No mocap data. Publisher time: {:.1f}'.format(
                    self.time_elapsed))
                pass

            try:
                x1, y1, yaw1 = self.tr1.get_values() # Get position.
                self.x1_old=self.x1_pos
                self.x1_pos=x1
                self.y1_old=self.y1_pos
                self.y1_pos=y1
                self.yaw1_old=self.yaw1_pos
                self.yaw1_pos=yaw1

            except:
                print("-----\nMissade forsta!")
                self.x1_pos=self.x1_old
                self.y1_pos=self.y1_old
                self.yaw1_pos=self.yaw1_old

                pass

            try:
                x2, y2, yaw2 = self.tr2.get_values() # Get position.
                self.x2_old=self.x2_pos
                self.x2_pos=x2
                self.y2_old=self.y2_pos
                self.y2_pos=y2
                self.yaw2_old=self.yaw2_pos
                self.yaw2_pos=yaw2

            except:
                print("-----\nMissade andra!")
                self.x2_pos=self.x2_old
                self.y2_pos=self.y2_old
                self.yaw2_pos=self.yaw2_old
                pass

            self.velocity(tid_mellan)

            self.v_tot1 = self.vma1.new_ma(self.v_tot1)
            self.v_tot2 = self.vma2.new_ma(self.v_tot2)
            #print("----------\nHastighet truck 1: {:.5f} \nHastighet truck 2: {:.5f}"
            #.format(self.v_tot1, self.v_tot2))
            # Publish data to the topic and sleep.
            self.pub.publish(self.x1_pos, self.y1_pos, self.yaw1_pos,
            self.x2_pos, self.y2_pos, self.yaw2_pos,
            self.time_elapsed, self.v_tot1, self.v_tot2)

            self.rate.sleep()



    def velocity(self,tid_mellan):
        """Calculates & Returns the velocities of the trucks.
        If dataloss occurs, the method will return the previous velocity."""

        if self.x1_pos==self.x1_old:
            try:
                v_x1=v_x1_old
                v_y1=v_y1_old
            except:
                v_x1=0
                v_y1=0
                pass
        else:
            v_x1=(self.x1_pos - self.x1_old)/tid_mellan
            v_y1=(self.y1_pos - self.y1_old)/tid_mellan
            v_x1_old = v_x1
            v_y1_old = v_y1

        if self.x2_pos==self.x1_old:
            #print("scener")
            try:
                v_x2 = v_x2_old
                v_y2 = v_y2_old
            except:
                v_x2 = 0
                v_y2 = 0
                pass
        else:
            v_x2=(self.x2_pos - self.x2_old)/tid_mellan
            v_y2=(self.y2_pos - self.y2_old)/tid_mellan
            v_x2_old = v_x2
            v_y2_old = v_y2



        self.v_tot1=math.sqrt(v_x1**2+v_y1**2)

        self.v_tot2=math.sqrt(v_x2**2+v_y2**2)

        return (self.v_tot1, self.v_tot2)


def main():
    mocap_used = False      # True if using Mocap, False if using simulation.
    freq = 20               # Publishing frequency in Hz. aa

    ma = 10#!/usr/bin/env python

    import rospy
    import curses
    import sys
    import os
    os.sys.path.append(
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
    import trucksender


    class CursesControl():
        """Class for controlling a truck with the keyboard. """
        def __init__(self, address, velocity_step = 50, angle_step = 50):
            self.vel_step = velocity_step
            self.ang_step = angle_step
            self.address = address

            self.init_velocity = 1500
            self.init_angle = 1500
            self.init_gearval = 60
            self.init_gear = 1

            self.angle_max = 1900
            self.angle_min = 1100

            self.velocity_max = 1900
            self.velocity_min = 1100

            self.velocity = self.init_velocity;
            self.angle = self.init_angle;
            self.gear = self.init_gear;
            self.gearval = self.init_gearval;

            self.sender = trucksender.TruckSender(address)

            self.stdscr = curses.initscr()


        def run(self):
            """Runs the curses window that captures keypresses. """
            curses.cbreak()
            self.stdscr.keypad(1)
            self.stdscr.refresh()

            self.stdscr.addstr(0, 10, 'Arrow keys or WASD to move truck')
            self.stdscr.addstr(1, 10, 'E to stop truck. Q to stop truck and quit'
                ' program.')
            self.stdscr.addstr(2, 10, 'Ctrl-C will quit but not stop truck.')
            self.stdscr.addstr(11, 20, '       ')

            self.set_velocity()
            self.set_angle()
            self.set_gear()

            key = ''
            while key != ord('q'):
                key = self.stdscr.getch()
                self.stdscr.refresh()

                if key == ord('e'):
                    self.reset()

                if key == curses.KEY_UP or key == ord('w'):
                    self.velocity = self.velocity - self.vel_step
                    self.set_velocity()

                elif key == curses.KEY_DOWN or key == ord('s'):
                    self.velocity = self.velocity + self.vel_step
                    self.set_velocity()

                if key == curses.KEY_LEFT or key == ord('a'):
                    self.angle = self.angle + self.ang_step
                    self.set_angle()

                elif key == curses.KEY_RIGHT or key == ord('d'):
                    self.angle = self.angle - self.ang_step
                    self.set_angle()

                if key == curses.KEY_NPAGE:
                    if(self.gear > 1):
                        self.gear = self.gear - 1;

                    self.set_gear()

                elif key == curses.KEY_PPAGE:
                    if(self.gear < 3):
                        self.gear = self.gear + 1;

                    self.set_gear()


            curses.endwin()
            self.sender.stop_truck()


        def reset(self):
            self.sender.stop_truck()

            self.velocity = self.init_velocity
            self.angle = self.init_angle
            self.gear = 1
            self.gearval = self.init_gearval

            self.set_velocity()
            self.set_angle()
            self.set_gear()

            self.stdscr.addstr(11, 30, 'stopped')


        def set_velocity(self):
            """Sets the velocity of the truck. """
            if self.velocity < self.velocity_min:
                self.velocity = self.velocity_min
            if self.velocity > self.velocity_max:
                self.velocity = self.velocity_max

            if self.velocity == self.init_velocity:
                txt = 'standing still  '
            elif self.velocity < self.init_velocity:
                txt = 'driving forward '
            else:
                txt = 'driving backward'

            self.stdscr.addstr(5, 20, txt)
            self.stdscr.addstr(5, 40, '%.2f' % self.velocity)
            self.stdscr.addstr(11, 30, '       ')

            self.sender.send_data(self.velocity, self.angle, self.gearval)


        def set_angle(self):
            """Sets the angle of the truck wheels. """
            if self.angle < self.angle_min:
                self.angle = self.angle_min
            if self.angle > self.angle_max:
                self.angle = self.angle_max

            if self.angle == self.init_angle:
                txt = 'straight     '
            elif self.angle < self.init_angle:
                txt = 'turning right'
            else:
                txt = 'turning left '

            self.stdscr.addstr(7, 20, txt)
            self.stdscr.addstr(7, 40, '%.2f' % self.angle)
            self.stdscr.addstr(11, 30, '       ')

            self.sender.send_data(self.velocity, self.angle, self.gearval)


        def set_gear(self):
            """Sets the gear of the truck. """
            if (self.gear == 1):
                self.gearval = 60
            elif (self.gear == 2):
                self.gearval = 140 	#120
            else:
                self.gearval = 220	#200

            self.stdscr.addstr(9, 20, 'gear')
            self.stdscr.addstr(9, 40, '%d' % self.gear)
            self.stdscr.addstr(11, 30, '       ')

            self.sender.send_data(self.velocity, self.angle, self.gearval)





def main():
    mocap_used = False      # True if using Mocap, False if using simulation.
    freq = 20               # Publishing frequency in Hz. aa
    moving_average_num = 1

    # Publisher node info.
    topic_name = 'truck_topic'
    topic_type = truckmocap
    node_name = 'truck_pub'

    # Create and run the publisher.
    publ = TruckPublisher(node_name = node_name, topic_type = topic_type,
        topic_name = topic_name, mocap_used = mocap_used, update_freq = freq,
        ma = moving_average_num)
    publ.talker()

if __name__ == '__main__':
    main()
