#!/usr/bin/env python

import rospy,random
from std_msgs.msg import *

pathlist=random.sample(range(100),20)
n=len(pathlist)
print(pathlist)


def pathplan():
    pub = rospy.Publisher('PathPlan', String, queue_size=10)
    rospy.init_node('PathPlanner', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i=0
    while not rospy.is_shutdown():

	if i != len(pathlist):
		pathref = "vinkel %s, vinkelhastighet %s, tid %s" % (pathlist[i],pathlist[n-1-i],rospy.get_time())
		rospy.loginfo(pathref)
		pub.publish(pathref)
		rate.sleep()
		i+=1
	else:
		i=0
		print("de over, borjar om")
		pathplan()

if __name__ == '__main__':
    try:
        pathplan()
    except rospy.ROSInterruptException:
        pass
