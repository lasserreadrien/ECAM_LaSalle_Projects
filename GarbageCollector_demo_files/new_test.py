#!/usr/bin/env python2

##################################
#
# choose_from_fixed_positions.py file, for the IT&Robotics Lab Project
#
# Author: Adrien Lasserre (adrien.lasserre@ecam.fr) and Gwenn
#
##################################

import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.msg import *

global bol
bol=False
global temporary
temporary=2

def callback_fun(data):
	rospy.loginfo("Here is the chosen scheme:")
	rospy.loginfo(data.choice)

	global bol
	global temporary
	rospy.loginfo("Temporary:")
	rospy.loginfo(temporary)
	
	if data.choice==temporary:
		bol=True
	else:
		bol=False

	temporary=data.choice


def talker():
	rospy.init_node('new_test') #setting up the node
	pub = rospy.Publisher('choose_positions', FixedChoice, queue_size=10)
	rospy.Subscriber('scheme', FixedChoice, callback_fun)
	r=rospy.Rate(1)

	global temporary
	global bol

	while not rospy.is_shutdown():
		if bol==False:

			if temporary==1:
				pub.publish(1)
				r.sleep()
				pub.publish(3)
				r.sleep()
				pub.publish(4)
				r.sleep()
				pub.publish(2)
			elif temporary==2:
				pub.publish(1)
				r.sleep()
				pub.publish(2)
				r.sleep()
			else:
				pub.publish(1)
		##rospy.sleep(10.)
		
if __name__ == '__main__':
	talker()
