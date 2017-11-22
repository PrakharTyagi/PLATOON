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
        self.velocity = 1

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
        self.mocap_body2 = self.mocap.get_id_from_name('truckVehicle2')

    def get_values(self):
        """Returns the truck state. """
        truck_state1 = self.mocap.get_body(self.mocap_body2)
        x = truck_state1['x']
        y = truck_state1['y']
        yaw = truck_state1['yaw']

        return x, y, yaw*math.pi/180


class TruckPublisher():
    """Publisher class. Keeps an instance of the mocap or simulation object. """
    def __init__(self):
        self.topic_name = 'truck2'
        self.topic_type = truckmocap
        self.node_name = 'truck_pub'
        self.queue_size = 10
        self.update_freq = 20
        self.mocap_used = False

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

            if self.mocap_used:
                x, y, yaw = self.tr.get_values()
            else:
                self.tr.update_pos(self.time_elapsed)
                x, y, yaw = self.tr.get_values()

            self.pub.publish(x, y, yaw)
            self.rate.sleep()


def main():
    publ = TruckPublisher()
    publ.talker()

if __name__ == '__main__':
    main()
