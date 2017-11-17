#!/usr/bin/env python

import rospy
import numpy
import sys

from modeltruck_platooning.srv import state
from modeltruck_platooning.msg import *

from mocap_source_2 import Mocap, Body





def initmocap(truck_id):
	
	#initialize mocap connection
	#mocap = Mocap(host = '130.237.43.61', info = 1)
	#mocap_body1 =mocap.get_id_from_name('TruckVehicle1')
	#mocap_body2 =mocap.get_id_from_name('TruckVehicle2')		
	#mocap_body3 =mocap.get_id_from_name('TruckVehicle3')
		
	msg=get_state(truck_id)
	print(msg)
	return(msg)
		
	#self.sender()
	rospy.spin()


def get_state(truck_id):
	
	if truck_id==1:
		truck_state1={"x": 0.23, "y": 5.41, "yaw": 1.23}
		#truck_state1 = mocap.get_body(mocap_body1)
		if truck_state1 == 'off':
			rospy.logwarn("truck 1 is not found")
		else:
			msg1 = VehicleState()
			msg1.x = truck_state1['x']
			msg1.y = truck_state1['y']
			msg1.yaw = truck_state1['yaw']			
			msg=msg1			
			
			
	elif truck_id==2:
		truck_state2 = mocap.get_body(mocap_body2)
		if truck_state2 == 'off':
			rospy.logwarn("truck 2 is not found")
		else:
			msg2 = VehicleState()
			msg2.x = truck_state2['x']
			msg2.y = truck_state2['y']
			msg2.yaw = truck_state2['yaw']
			msg=msg2
			
	elif truck_id==3:
		truck_state3 = mocap.get_body(mocap_body3)
		if truck_state3 == 'off':
			rospy.logwarn("truck 3 is not found")
		else:
			msg3 = VehicleState()
			msg3.x = truck_state3['x']
			msg3.y = truck_state3['y']
			msg3.yaw = truck_state3['yaw']
			msg=msg3
			
	else:
		print("No such truck")
		truck_state_server()
		
	return (msg)
	
def handle_truck_state(truck_id):
	truck=initmocap(truck_id.truck_id)
	#print(truck.x,truck.y,truck.yaw)

	print "Returning state of truck %s: %s, %s, %s"%(truck_id.truck_id,truck.x, truck.y, truck.yaw)
	return (truck.x, truck.y, truck.yaw)		
		
def truck_state_server():
	rospy.init_node('truck_state_server')
	s = rospy.Service('truck_state', state, handle_truck_state) #service name, servicetype, handle
	print "Ready to send truckstate."
	rospy.spin()

if __name__ == "__main__":
	truck_state_server()
	
