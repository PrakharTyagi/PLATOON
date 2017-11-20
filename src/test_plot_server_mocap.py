#!/usr/bin/env python

from modeltruck_platooning.srv import *
from mocap_source_2 import Mocap
import rospy
import time
import math

class Truck:
    def __init__(self):
        #initialize mocap connection
        self.mocap = Mocap(host = '192.168.1.10', info = 1)
        self.mocap_body1 = self.mocap.get_id_from_name('TruckVehicle2')

    def get_values(self):
        truck_state1 = self.mocap.get_body(self.mocap_body1)
        x = truck_state1['x']
        y = truck_state1['y']
        yaw = truck_state1['yaw']

        return x, y, yaw*math.pi/180


class PlotServer():
    def __init__(self):
        self.time_elapsed = 0
        self.init_time = time.time()
        self.truck = Truck()

    def handle_request(self, req):

        time_new = time.time()
        self.time_elapsed = time_new - self.init_time

        x, y, yaw = self.truck.get_values()

        # print(
        #     'Returning x = {:07.4f}, y = {:07.4f}, yaw = {:07.4f}, time = {:.1f}'.format(
        #     x, y, yaw, self.time_elapsed))

        return TestPlotResponse(x, y, yaw, self.time_elapsed)

    def test_plot_server(self):
        rospy.init_node('test_plot_server')
        s = rospy.Service('test_plot', TestPlot, self.handle_request)
        print('Ready to return values')
        rospy.spin()

if __name__ == '__main__':
    plotsrv = PlotServer()
    plotsrv.test_plot_server()
