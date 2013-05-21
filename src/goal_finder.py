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
	# quaternion = quaternion_from_euler(data.roll,data.pitch,data.yaw)
	#(data.x_center/1000.0,data.y_center/1000.0,data.z_center/1000.0)
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

'''
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
'''


if __name__ == '__main__':
	inkey_buffer = 0
	rospy.init_node('goal_finder', anonymous=True)

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)

	#ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
	
	#ac.wait_for_server()
	pub = rospy.Publisher('/arty/move_base/goal',MoveBaseActionGoal)
	goal_location = MoveBaseActionGoal()



	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/head_pose', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([rot[0], rot[1], rot[2], rot[3]])

		theta = math.pi/2 + pitch
		x = -trans[2]*math.tan(theta)
		y = x*math.tan(-yaw)
		print "dsf"
		try:
			character = raw_input("press key to set goal location...")
			print "x = ", x, "    y = ", y
			goal_location.goal.target_pose.pose.position.x = x
			goal_location.goal.target_pose.pose.position.y = y
			goal_location.goal.target_pose.pose.orientation.w = 1.0
			goal_location.goal.target_pose.header.frame_id = 'head_pose'
			goal_location.goal.target_pose.header.stamp = rospy.Time.now()
			pub.publish(goal_location)
			#ac.send_goal(goal)
			#ac.wait_for_result()
			#print ac.get_result()


		except rospy.ROSInterruptException:
			print "Interrupt from keyboard"

		# inkey_buffer=int(random.random()*2)
		# character = inkey()
		# try:
		# 	if character == "a":
		# 		print "pressed a key"
		# 		print "x = ", x,", y = ",y
		# 	if character == "e":
		# 		exit()
		# except (KeyboardInterrupt, SystemExit):
		# 	raise
		# 	print 'Press Ctrl+C'

		# signal.signal(signal.SIGINT, signal_handler)
		# print 'Press Ctrl+C'
		# signal.pause()