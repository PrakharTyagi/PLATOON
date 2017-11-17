#!/usr/bin/env python

'''
skicka tillbaka
'''

from modeltruck_platooning.srv import * #gabsrv.srv
from modeltruck_platooning.msg import VehicleState
import rospy

def handle_state_data(req):

    print "State [%s + %s + %s]"%(req.x, req.y, req.yaw)

    return TruckState(req.x, req.y, req.yaw)

def state_server():
    rospy.init_node('state_server')
    s = rospy.Service('truck_state', gabsrv, handle_state_data) #(name, servicetype, handle)
    print "Ready to send stateinfo."
    rospy.spin()

if __name__ == "__main__":
    state_server()
