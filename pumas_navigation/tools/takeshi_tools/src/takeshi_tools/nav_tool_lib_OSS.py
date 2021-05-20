#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray, Bool, Empty
from geometry_msgs.msg import Pose, Vector3, Quaternion, PoseStamped

global globalGoalReached
global goalReached
global omnibase
global pubGlobalGoalXYZ
global whole_body
global robot_stop
globalGoalReached=True
goalReached=True
robot_stop=False

def callback_global_goal_reached(msg):
	global globalGoalReached
	globalGoalReached = msg.data

def callback_goal_reached(msg):
	global goalReached
	goalReached = msg.data

def callback_stop(msg):
	global robot_stop
	robot_stop=True


class nav_module():
	"""docstring for nav_module"""
	
	def __init__(self):
		self.goal=Float32MultiArray()
		self.pubGlobalGoalXYZ = rospy.Publisher('/navigation/mvn_pln/get_close_xya', Float32MultiArray, queue_size=1)
		self.pubDistAngle = rospy.Publisher('/navigation/path_planning/simple_move/goal_dist_angle', Float32MultiArray, queue_size=1)
		self.pubMoveRel = rospy.Publisher('/navigation/path_planning/simple_move/goal_move_rel', Float32MultiArray, queue_size=1)
		self.pubRobotStop = rospy.Publisher('/hardware/robot_state/stop', Empty, queue_size=1)
		rospy.Subscriber("/navigation/global_goal_reached", Bool, callback_global_goal_reached)
		rospy.Subscriber("/navigation/goal_reached", Bool, callback_goal_reached)
		rospy.Subscriber("/hardware/robot_state/stop", Empty, callback_stop)
		

	def getClose(self,x,y,theta, timeout=0.0):
		global globalGoalReached
		global robot_stop
		rate=rospy.Rate(10)
		goal=Float32MultiArray()
		globalGoalReached=False
		robot_stop=False
		goal.data=[x,y,theta]
		if timeout!=0:
			attemps=int(timeout*10)
		else:
			attemps=10000000000000
		#print  goal.data
		self.pubGlobalGoalXYZ.publish(goal)
		rate.sleep()
		rospy.sleep(5.)
		while not globalGoalReached and not rospy.is_shutdown() and not robot_stop and attemps>=0:
			attemps-=1
			rate.sleep()
		robot_stop=False
		if not globalGoalReached:
			msg_stop=Empty()
			self.pubRobotStop.publish(msg_stop)
			rate.sleep()
			rospy.sleep(5.)
		x=rospy.Duration.from_sec(2.5)
		rospy.sleep(x)	


	def moveDistAngle(self,x, yaw, timeout=0.0):
		global goalReached
		global robot_stop
		rate=rospy.Rate(10)
		goal=Float32MultiArray()
		goalReached =False
		robot_stop=False
		goal.data=[x,yaw]
		#print  goal.data
		if timeout!=0:
			attemps=int(timeout*10)
		else:
			attemps=10000000000000
		self.pubDistAngle.publish(goal)
		rate.sleep()
		rospy.sleep(5.)
		while not goalReached and not rospy.is_shutdown() and not robot_stop and attemps>=0:
			attemps-=1
			rate.sleep()
		robot_stop=False
		if not goalReached:
			msg_stop=Empty()
			self.pubRobotStop.publish(msg_stop)
			rate.sleep()
			rospy.sleep(5.)
		rate.sleep()
		x=rospy.Duration.from_sec(2.5)
		rospy.sleep(x)


	def go_dist_angle(self, x=0.0, yaw=0.0,timeout=0.0):
		self.moveDistAngle(x,yaw,timeout)

	def moveRel(self,x, y, yaw, timeout=0.0):
		global goalReached
		global robot_stop
		rate=rospy.Rate(10)
		goal=Float32MultiArray()
		goalReached =False
		robot_stop=False
		goal.data=[x,y,yaw]
		#print  goal.data
		if timeout!=0:
			attemps=int(timeout*10)
		else:
			attemps=10000000000000
		self.pubMoveRel.publish(goal)
		rate.sleep()
		rospy.sleep(5.)
		while not goalReached and not rospy.is_shutdown() and not robot_stop and attemps>=0:
			attemps-=1
			rate.sleep()
		robot_stop=False
		if not goalReached:
			msg_stop=Empty()
			self.pubRobotStop.publish(msg_stop)
			rate.sleep()
			rospy.sleep(5.)
		rate.sleep()
		x=rospy.Duration.from_sec(2.5)
		rospy.sleep(x)
