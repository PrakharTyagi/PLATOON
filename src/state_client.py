#!/usr/bin/env python

import sys
import rospy
from modeltruck_platooning.srv import *

def get_state_client(x, y, yaw):
    rospy.wait_for_service('truck_state')#waits until service is available
    try:
        truck_state = rospy.ServiceProxy('truck_state', gabsrv) 
        resp1 = truck_state(x, y, yaw)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s = [x y yaw] "%sys.argv[1:4] #sys.argv lista med argumenten du skickar in t servern

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
	yaw=float(sys.argv[3])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s, %s, %s"%(x, y, yaw)
    print "x:%s + y:%s  yaw:%s"%(get_state_client(x, y, yaw))
