#!/usr/bin/env python

from modeltruck_platooning.srv import *
import rospy
import time
import math

class CircleTruck():
    def __init__(self):
        self.radius = 2
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


class PlotServer():
    def __init__(self):
        self.time_elapsed = 0
        self.init_time = time.time()
        self.ct = CircleTruck()

    def handle_request(self, req):

        time_new = time.time()
        self.time_elapsed = time_new - self.init_time

        self.ct.update_pos(self.time_elapsed)
        x, y, yaw = self.ct.get_values()

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
