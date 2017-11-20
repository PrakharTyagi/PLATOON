#!/usr/bin/env python

import sys
import rospy
from modeltruck_platooning.srv import state, stateRequest
from modeltruck_platooning.msg import *

def get_truck_state(truck_id):
	rospy.wait_for_service('truck_state')#waits until service is available
	try:
		truck_state = rospy.ServiceProxy('truck_state', state)
		response1 = truck_state(1)
		#response2 = truck_state(2)
		#resp2 = truck_state(truck_id)

		#print(resp1)
		return response1 # response2

	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def usage():
	return "Truck %s "%sys.argv[0]

if __name__ == "__main__":
	if len(sys.argv) == 2:
		truck_id = int(sys.argv[1])
		#truck_id2 = int(sys.argv[2])

	else:
		print usage()
		sys.exit(1)
	print "Requesting state of truck %s" % (int(truck_id))
	if get_truck_state(truck_id) == None:
		print("No such truck m8")
	else:
		print ("State truck %s : %s" % (truck_id, get_truck_state(truck_id)))
