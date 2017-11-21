#!/usr/bin/env python

import sys
import rospy
from modeltruck_platooning.srv import *
import time


if __name__ == '__main__':
    try:
        rospy.wait_for_service('test_plot', timeout = 2)
        test_plot = rospy.ServiceProxy('test_plot', TestPlot)

    except rospy.exceptions.ROSException as e:
        print('Error when connecting to service: {}'.format(e))
        sys.exit(1)

    try:
        while True:
            resp = test_plot()
            print('whooo {}, {}, {}, {}'.format(
                    resp.x, resp.y, resp.yaw, resp.timestamp))
            time.sleep(0.01)

    except rospy.ServiceException, e:
        print('Service call failed: {}'.format(e))
        sys.exit(1)
