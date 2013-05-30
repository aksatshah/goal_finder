#!/usr/bin/env python
import roslib; roslib.load_manifest('goal_finder')
import rospy
import tf
import math
import termios
import tty
import sys
import random
import actionlib
import signal
from move_base_msgs.msg import *
from std_msgs.msg import String
from head_pose.msg import head_data
from geometry_msgs.msg import Twist
from tf.transformations import *

global inkey_buffer

def converter(pub,data):
	move = Twist()
	if (data.pitch > 0.15) and (data.pitch < 1.0): #looking down
		move.linear.x = 1.0
	if (data.pitch < -0.4) and (data.pitch > -1.0): #looking up
		move.linear.x = -1.0
	if (data.yaw > 0.4) and (data.yaw < 1.0): #looking left
		move.angular.z = 1.0
	if (data.yaw < -0.4) and (data.yaw > -1.0): #looking right
		move.angular.z = -1.0
	print(move)
	pub.publish(move)


def callback(data):
	pub = rospy.Publisher('cmd_vel',Twist)
	converter(pub,data)
	head_tf = tf.TransformBroadcaster()
	head_tf.sendTransform((data.x_center/1000.0,-data.y_center/1000.0,data.z_center/1000.0),
		quaternion_from_euler(0,data.pitch,math.pi+data.yaw), #roll,pitch,yaw
		rospy.Time.now(),
		"head_pose",
		"xtion")
	rospy.loginfo("Time to move!")
	#print(data)

def inkey():
		fd=sys.stdin.fileno()
		remember_attributes=termios.tcgetattr(fd)
		tty.setraw(sys.stdin.fileno())
		character=sys.stdin.read(inkey_buffer)
		termios.tcsetattr(fd, termios.TCSADRAIN, remember_attributes)
		return character

def signal_handler(signal, frame):
        print 'You pressed Ctrl+C!'
        sys.exit(0)


if __name__ == '__main__':
	inkey_buffer = 0
	rospy.init_node('goal_finder', anonymous=True)

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)

	pub = rospy.Publisher('/arty/move_base/goal',MoveBaseActionGoal)
	pub_reverse = rospy.Publisher('/arty/cmd_vel', Twist)
	goal_location = MoveBaseActionGoal()

	while not rospy.is_shutdown():
		character = raw_input("press key to set goal location...")
		try:
			(trans,rot) = listener.lookupTransform('/head_pose', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])
		
		print
		print "roll = ", roll
		print
		print "pitch = ", pitch
		print
		print "yaw = ", yaw
		print

		if pitch < -0.05:
			print "going forward..."
			theta = math.pi/2 + pitch
			x = -trans[2]*math.tan(theta)
			y = x*math.tan(-yaw)

			print "x = ", x, "    y = ", y
			if x < 3.0:
				goal_location.goal.target_pose.pose.position.x = x
				goal_location.goal.target_pose.pose.position.y = -y
				goal_location.goal.target_pose.pose.orientation.x = 0.0
				goal_location.goal.target_pose.pose.orientation.y = 0.0
				goal_location.goal.target_pose.pose.orientation.z = rot[2]
				goal_location.goal.target_pose.pose.orientation.w = rot[3]
				goal_location.goal.target_pose.header.frame_id = 'base_link'
				goal_location.goal.target_pose.header.stamp = rospy.Time.now()
				pub.publish(goal_location)
			else:
				print "Your are looking too far ahead - please look down"
		else:
			move = Twist()
			if yaw < -0.25:
				move.angular.z = 1.0
			elif yaw > 0.25:
				move.angular.z = -1.0
			else:
				move.linear.x=-1.0
			print move
			pub_reverse.publish(move)

		#pub.publish(goal_location)