#!/usr/bin/env python

import rospy
import numpy
import sys

from modeltruck_platooning.srv import state
from modeltruck_platooning.msg import *

from mocap_source_2 import Mocap, Body




class truck_state:
	def __init__(self,truck_id):
		#print(truck_id)

		#initialize mocap connection
		#self.mocap = Mocap(host = '130.237.43.61', info = 1)
		

		#rospy.init_node('mocap_sender', anonymous = True)
		#self.pub1 = rospy.Publisher('Mocapstate1', VehicleState, queue_size = 1)
		#self.pub2 = rospy.Publisher('Mocapstate2', VehicleState, queue_size = 1)
		#self.pub3 = rospy.Publisher('Mocapstate3', VehicleState, queue_size = 1)

		#self.mocap_body1 =self.mocap.get_id_from_name('TruckVehicle1')
		#self.mocap_body2 =self.mocap.get_id_from_name('TruckVehicle2')
		#self.mocap_body3 =self.mocap.get_id_from_name('TruckVehicle3')
		#self.mocap_body2 = self.mocap.get_id_from_name('MiniTruck2')
		#self.mocap_body3 = self.mocap.get_id_from_name('MiniTruck3')
		
		self.get_state(truck_id)
		
		#self.sender()
		rospy.spin()


	def get_state(self,truck_id):
		#print("Tjena %s" % truck_id)
		if truck_id==1:
			truck_state1={"x": "x", "y": "y", "yaw": "yaw"}
			#truck_state1 = self.mocap.get_body(self.mocap_body1)
			if truck_state1 == 'off':
				rospy.logwarn("truck 1 is not found")
			else:
				msg1 = VehicleState()
				msg1.x = truck_state1['x']
				msg1.y = truck_state1['y']
				msg1.yaw = truck_state1['yaw']
				
				msg=msg1
				print("tja  \n%s" % msg)
			
				#return (msg)
				#print("tja \n%s" % msg.x)			
				
				
		elif truck_id==2:
			truck_state2 = self.mocap.get_body(self.mocap_body2)
			if truck_state2 == 'off':
				rospy.logwarn("truck 2 is not found")
			else:
				msg2 = VehicleState()
				msg2.x = truck_state2['x']
				msg2.y = truck_state2['y']
				msg2.yaw = truck_state2['yaw']
				msg=msg2
				

		elif truck_id==3:
			truck_state3 = self.mocap.get_body(self.mocap_body3)
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
		

def handle_truck_state(truck_id):
	truck=truck_state(truck_id.truck_id).get_state(truck_id.truck_id)
	print(truck)
	#print(msg)
	print("halloj %s" % truck_id)
	#print "Returning state of truck %s: %s, %s, %s"%(truck_id.truck_id, msg,msg,msg)
	return (truck)		
		
def truck_state_server():
	rospy.init_node('truck_state_server')
	s = rospy.Service('truck_state', state, handle_truck_state) #service name, servicetype, handle
	print "Ready to send truckstate."
	rospy.spin()

if __name__ == "__main__":
	truck_state_server()
	
