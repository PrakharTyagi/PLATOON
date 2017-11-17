#!/usr/bin/env python
import rospy

from modeltruck_platooning.msg import VehicleState

xlist=[]
ylist=[]
anglelist=[]

def truckstate():
	rospy.init_node('truckstate', anonymous=True)
	rospy.Subscriber("Mocapstate1",VehicleState,callback)
	rospy.spin()

def callback(data):
	x = data.x
	y = data.y
	angle = data.yaw

	xlist.append(x)
	ylist.append(y)
	anglelist.append(angle)
	
	print('x_coordinate:', x)
	print('y_coordinate:', y)
	print('Angle :', angle)
	
	#fixa service där du specifierar när datainhämtningarna ska ske
	#om inga nya värden registreras efter ett antal "samplingar", skickas datalistorna
	#till log() där värdena loggas i en txtfil
	
	if len(xlist) < 6:
		pass
	elif xlist[end-5]==xlist[end]:
		log(xlist,ylist,anglelist)            
	else:
		pass
	
def log(xlist,ylist,anglelist):
	fil=open("mologg.txt","w") #se till att en txtfil med rätt namn finns i samma mapp
	for i in xlist:
		fil.write(str(xlist[i-1])+" "+str(ylist[i-1])+" "+str(anglelist[i-1])+'\n')
	fil.close()        

if __name__ == '__main__':
    truckstate()


