#!/usr/bin/env python2

##################################
#
# choose_from_fixed_positions.py file, for the IT&Robotics Lab Project
#
# Author: Adrien Lasserre (adrien.lasserre@ecam.fr)
#
##################################

import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.msg import *

desired_values = [1, 650, 2, 440, 3, 430, 4, 545] 

def callback_func(data):
	#match case for the different fixed positions
	rospy.loginfo("Here's the choice:")
	rospy.loginfo(data.choice)

	tmp=data.choice

	global desired_values

	if tmp == 1:
		desired_values = [1, 650, 2, 440, 3, 430, 4, 545] #arm perfectly horizontal
		#pub_msg(desired_values)
	elif tmp == 2:
		desired_values = [1, 650, 2, 440, 3, 430, 4, 590] #same but gripped
		#pub_msg(desired_values)
	elif tmp == 3:
		desired_values = [1, 650, 2, 787, 3, 430, 4, 545]
		#pub_msg(desired_values)
	elif tmp == 4:
		desired_values = [1, 650, 2, 787, 3, 430, 4, 590]
		#pub_msg(desired_values)
	elif tmp == 5:
		desired_values = [1, 717, 2, 440, 3, 430, 4, 545]
		#pub_msg(desired_values)
	elif tmp == 6:
		desired_values = [1, 717, 2, 440, 3, 430, 4, 590]
		#pub_msg(desired_values)
	else:
		desired_values = [1, 650, 2, 440, 3, 430, 4, 545] #arm perfectly horizontal
		#pub_msg(desired_values)

def talker():
	rospy.init_node('choose_from_fixed_positions') #setting up the node
	pub = rospy.Publisher('set_position', SetPosition, queue_size=100)#setting up the Publisher
	rospy.Subscriber('choose_positions', FixedChoice, callback_func) #setting up the Subscriber
	r=rospy.Rate(10)

	#publish the messages
	#desired_values = [1, 562, 2, 469, 3, 430, 4, 545] #arm perfectly horizontal

	#definition of the message to be published on the /set_position topic

	global desired_values

	while not rospy.is_shutdown():

		#second motor
		pub.publish(desired_values[2], desired_values[3])
		r.sleep()
		#third motor
		pub.publish(desired_values[4], desired_values[5])
		r.sleep()
		#fourth motor
		pub.publish(desired_values[6], desired_values[7])
		r.sleep()
		#first motor
		pub.publish(desired_values[0], desired_values[1])
		r.sleep()
		#rospy.spin()#waiting for inputs from /choose_positions topic



if __name__ == '__main__':
	talker()
