#!/usr/bin/env python

import rospy
#from std_msgs.msg import String
from modeltruck_platooning.msg import *
import time
import math
from mocap_source_2 import *

class CircleTruck():
    def __init__(self):
        self.radius = 1.3
        self.theta0 = 0
        self.theta = self.theta0
        self.x = self.radius*math.cos(self.theta)
        self.y = self.radius*math.sin(self.theta)
        self.velocity = 1
        self.yaw = self.theta + math.pi/2

    def update_pos(self, time_elapsed):
        self.theta = self.theta0 + time_elapsed*self.velocity/self.radius

        self.x = self.radius*math.cos(self.theta)
        self.y = self.radius*math.sin(self.theta)
        self.yaw = (self.theta + math.pi/2) % (2*math.pi)

    def get_values(self):
        return self.x, self.y, self.yaw

class Truck:
    def __init__(self):
        #initialize mocap connection
        self.mocap = Mocap(host = '192.168.1.10', info = 1)
        #self.mocap_body1 = self.mocap.get_id_from_name('TruckVehicle2')
        self.mocap_body2 = self.mocap.get_id_from_name('truckVehicle2')

    def get_values(self):
        truck_state1 = self.mocap.get_body(self.mocap_body2)
        x = truck_state1['x']
        y = truck_state1['y']
        yaw = truck_state1['yaw']

        return x, y, yaw*math.pi/180


class TruckPublisher():
    def __init__(self):
        self.pub = rospy.Publisher('truck2', truckmocap, queue_size=10)
        rospy.init_node('truck2', anonymous=True)
        self.rate = rospy.Rate(20)
        self.time_elapsed = 0
        self.init_time = time.time()

        self.mocap_used = True

        if self.mocap_used:
            self.tr = Truck()
        else:
            self.tr = CircleTruck()

        print('Publisher running')

    def talker(self):
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
    # try:
    #     talker()
    # except rospy.ROSInterruptException:
    #     pass
