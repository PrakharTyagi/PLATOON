#!/usr/bin/env python

import sys
import rospy
from modeltruck_platooning.srv import state.srv

def Truck_State_client(truck_id):
    rospy.wait_for_service('Truck_State')#waits until service is available
    try:
        Truck_State = rospy.ServiceProxy('Truck_State', state)
        resp1 = Truck_State(truck_id)
        return resp1

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y yaw]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        truck_id = float(sys.argv[1])
        
    else:
        print usage()
        sys.exit(1)
    print "Requesting state of truck %s+%s"%(truck_id)
    print "State of truck %s is %s"%(truck_id, Truck_State_client(truck_id))