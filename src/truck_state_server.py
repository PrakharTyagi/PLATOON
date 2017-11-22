#!/usr/bin/env python

import rospy
import numpy
import sys
import time
import math

from modeltruck_platooning.srv import state, stateResponse, stateRequest
from modeltruck_platooning.msg import *

from mocap_source_2 import Mocap, Body




class truck_state:
	def __init__(self):


		#initialize mocap connection
		self.mocap = Mocap(host = '192.168.1.10', info = 0)


		#rospy.init_node('mocap_sender', anonymous = True)
		#self.pub1 = rospy.Publisher('Mocapstate1', VehicleState, queue_size = 1)
		#self.pub2 = rospy.Publisher('Mocapstate2', VehicleState, queue_size = 1)
		#self.pub3 = rospy.Publisher('Mocapstate3', VehicleState, queue_size = 1)

		#self.mocap_body1 =self.mocap.get_id_from_name('TruckVehicle1')
		self.mocap_body2 =self.mocap.get_id_from_name('truckVehicle2')
		#self.mocap_body3 =self.mocap.get_id_from_name('TruckVehicle2')
		#self.mocap_body2 = self.mocap.get_id_from_name('MiniTruck2')
		#self.mocap_body3 = self.mocap.get_id_from_name('MiniTruck3')

		rospy.init_node('truck_state_server')
		s = rospy.Service('truck_state', state, self.get_state) #service name, servicetype, handle
		print "Ready to send truckstate."

		rospy.spin()

		#self.sender()


	def get_state(self,request):

		truck_id = request.truck_id
		#print(truck_id)
		if truck_id==1:
			#truck_state1={"x": 1.2, "y": 0.3, "yaw": 4.4}
			truck_state1 = self.mocap.get_body(self.mocap_body1)
			if truck_state1 == 'off':
				rospy.logwarn("truck 1 is not found")
			else:
				msg = stateResponse()
				msg.x = truck_state1['x']
				msg.y = truck_state1['y']
				msg.yaw = truck_state1['yaw']*math.pi/180
				#time.sleep(5)
				print("Returning state of truck 1: %s %s %s" % (msg.x,msg.y,msg.yaw))
				return msg


		elif truck_id==2:
			#truck_state2 = {"x": 1.2, "y": 0.3, "yaw": 4.4}
			truck_state2 = self.mocap.get_body(self.mocap_body2)
			if truck_state2 == 'off':
				rospy.logwarn("truck 2 is not found")
			else:
				msg = stateResponse()
				msg.x = truck_state2['x']
				msg.y = truck_state2['y']
				msg.yaw = truck_state2['yaw']*math.pi/180

				print("Returning state of truck 2: %s %s %s" % (msg.x,msg.y,msg.yaw))

				return msg

		elif truck_id==3:
			truck_state3 = self.mocap.get_body(self.mocap_body3)
			if truck_state3 == 'off':
				rospy.logwarn("truck 3 is not found")
			else:
				msg = stateResponse()
				msg.x = truck_state3['x']
				msg.y = truck_state3['y']
				msg.yaw = truck_state3['yaw']

				print("Returning state of truck 3: %s %s %s" % (msg.x,msg.y,msg.yaw))

				return msg

		else:
			print("No such truck")
			pass
		#print("Returning state of truck %s: %s %s %s" % (truck_id,msg.x,msg.y,msg.yaw))


if __name__ == "__main__":
	sender = truck_state()
