#!/usr/bin/env python2

import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.msg import *
from std_msgs.msg import *

global choix
choix = 2
global tmp
tmp = False

def clb(data):
	global choix
	global tmp
	tmp=data.data

	rospy.loginfo(tmp)

	if tmp:
		choix = 1
	else:
		choix = 2

def talker():
	rospy.init_node('demonstration')
	pub=rospy.Publisher('scheme', FixedChoice, queue_size=10)
	rospy.Subscriber('choose_archi', Bool, clb)
	r=rospy.Rate(1)

	global choix

	while not rospy.is_shutdown(): 
		pub.publish(choix)
		r.sleep()

if __name__ == '__main__':
	talker()
