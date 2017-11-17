#!/usr/bin/env python

import sys
import rospy
from modeltruck_platooning.srv import *

def test_plot_client():
    rospy.wait_for_service('test_plot')
    try:
        test_plot = rospy.ServiceProxy('test_plot', TestPlot)
        resp1 = test_plot()
        return resp1.x, resp1.y, resp1.yaw, resp1.timestamp
    except rospy.ServiceException, e:
        print('Service call failed: {}'.format(e))


if __name__ == '__main__':
    x, y, yaw, ts = test_plot_client()
    print('whooo {}, {}, {}, {}'.format(x, y, yaw, ts))
