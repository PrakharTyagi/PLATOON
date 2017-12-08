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
    def __init__(self,
        omega = 0.75, theta0 = 0, radius = [1.3, 1.3], offset = [0, 0]):

        self.omega = omega          # Angular velocity.
        self.theta0 = theta0        # Initial angle.
        self.xradius = radius[0]    # x-radius of ellipse path.
        self.yradius = radius[1]
        self.xc = offset[0]         # x-coordinate of ellipse path center.
        self.yc = offset[1]

        self.theta = self.theta0            # Angle for position on circle.
        self.x = self.xc + self.xradius*math.cos(self.theta)
        self.y = self.yc + self.yradius*math.sin(self.theta)
        self.yaw = self.theta + math.pi/2   # Angle of truck.

    def update_pos(self, time_elapsed):
        """Update the position of the simulated truck. """
        self.theta = self.theta0 + time_elapsed*self.omega

        self.x = self.xc + self.xradius*math.cos(self.theta)
        self.y = self.yc + self.yradius*math.sin(self.theta)
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
                 queue_size = 1, ma = 1,
                 simw = 0.75, simr = [1.3, 1.3], simc = [0, 0]):
        self.node_name = node_name
        self.topic_type = topic_type
        self.topic_name = topic_name
        self.update_freq = update_freq
        self.mocap_used = mocap_used
        self.queue_size = queue_size

        circletruck_rad_distance = -0.4

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
            self.tr1 = CircleTruck(simw, 0, simr, simc)
            self.tr2 = CircleTruck(
                simw, circletruck_rad_distance, simr, simc)

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


            if not self.mocap_used:
                try:
                    self.tr1.update_pos(self.time_elapsed)
                    self.tr2.update_pos(self.time_elapsed)
                except:
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
    mocap_used = True      # True if using Mocap, False if using simulation.
    freq = 20               # Publishing frequency in Hz. aa
    moving_average_num = 1

    # Publisher node info.
    topic_name = 'truck_topic'
    topic_type = truckmocap
    node_name = 'truck_pub'

    simw = 0.75         # Angular velocity of simulated trucks.
    simr = [1.7, 1.2]   # Radii of simulated trucks ellipse path.
    simc = [0.3, -1.3]  # Center of simulated trucks ellipse path.

    # Use simulation if entered 0 as argument. Otherwise use value above.
    try:
        if int(sys.argv[1]) == 0:
            mocap_used = False
    except:
        pass

    # Create and run the publisher.
    publ = TruckPublisher(node_name = node_name, topic_type = topic_type,
        topic_name = topic_name, mocap_used = mocap_used, update_freq = freq,
        ma = moving_average_num, simw = simw, simr = simr, simc = simc)
    publ.talker()

if __name__ == '__main__':
    main()
