#!/usr/bin/env python

import rospy
from modeltruck_platooning.msg import *
import time
import math
from mocap_source_2 import *

class CircleTruck():
    """Class for simulating a truck driving in a circle. """
    def __init__(self):
        self.radius = 1.3
        self.theta0 = 0     # Initial angle.
        self.velocity = 2

        self.theta = self.theta0
        self.x = self.radius*math.cos(self.theta)
        self.y = self.radius*math.sin(self.theta)
        self.yaw = self.theta + math.pi/2

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
    def __init__(self):
        self.mocap = Mocap(host = '192.168.1.10', info = 1)
        self.mocap_body2 = self.mocap.get_id_from_name('TruckVehicle2')

    def get_values(self):
        """Returns the truck state. """
        truck_state1 = self.mocap.get_body(self.mocap_body2)
        x = truck_state1['x']
        y = truck_state1['y']
        yaw = truck_state1['yaw']

        return x, y, yaw*math.pi/180


class TruckPublisher():
    """Publisher class. Keeps an instance of the mocap or simulation object. """
    def __init__(self, topic_name = 'truck2', topic_type = truckmocap,
                node_name = 'truck_pub', update_freq = 20, mocap_used = True,
                queue_size = 10):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.node_name = node_name
        self.queue_size = queue_size
        self.update_freq = update_freq
        self.mocap_used = mocap_used
        #self.vtot=0
        self.time_bw=[0]
        self.xlist=[0]
        self.ylist=[0]

        self.pub = rospy.Publisher(self.topic_name, self.topic_type,
            queue_size = self.queue_size)
        rospy.init_node(self.node_name, anonymous = True)

        self.rate = rospy.Rate(self.update_freq)
        self.time_elapsed = 0
        self.init_time = time.time()

        if self.mocap_used:
            self.tr = Truck()
        else:
            self.tr = CircleTruck()

        print('Publisher running')

    def talker(self):
        """Publishes the mocap or simulated data continuously to the topic. """
        while not rospy.is_shutdown():
            time_new = time.time()
            self.time_elapsed = time_new - self.init_time
            self.time_bw.append(rospy.get_time())
            tid_mellan=(self.time_bw[-1]-self.time_bw[-2])
            #print(tid_mellan)
            try:

                if self.mocap_used:
                    x, y, yaw = self.tr.get_values()
                else:
                    self.tr.update_pos(self.time_elapsed)
                    x, y, yaw = self.tr.get_values()

                self.xlist.append(x)
                self.ylist.append(y)
                self.velocity(tid_mellan)
                self.pub.publish(x, y, yaw, self.time_elapsed,self.v_tot)
                #print("Speed %s " % self.v_tot)
                self.rate.sleep()
            except:
                pass


    def velocity(self,tid_mellan):
        v_x=(self.xlist[-1]-self.xlist[-2])/tid_mellan
        v_y=(self.ylist[-1]-self.ylist[-2])/tid_mellan

        self.v_tot=math.sqrt(v_x**2+v_y**2)

        return self.v_tot


def main():
    mocap_used = True      # True if using Mocap, False if using simulation.
    freq = 20               # Publishing frequency in Hz.
    # Publisher info
    topic_name = 'truck2'
    topic_type = truckmocap
    node_name = 'truck_pub'
    queue_size = 1

    publ = TruckPublisher(mocap_used = mocap_used, topic_name = topic_name,
        topic_type = topic_type, node_name = node_name, update_freq = freq,
        queue_size = queue_size)
    publ.talker()

if __name__ == '__main__':
    main()
