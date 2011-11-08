#! /usr/bin/env python
import roslib; roslib.load_manifest('youbot_haptic')
import rospy
from geometry_msgs.msg import *

class haptic_client():
	def __init__(self):
		# parameter
		self.base_velocity = geometry_msgs.msg.Twist()
		self.base_velocity.linear.x = 0.0
		self.base_velocity.linear.y = 0.0 
		self.base_velocity.linear.z = 0.0
		self.base_velocity.angular.x = 0.0
		self.base_velocity.angular.y = 0.0
		self.base_velocity.angular.z = 0.0
		
		# topics
		self.base_velocity_publisher = rospy.Publisher('/base_controller/command', geometry_msgs.msg.Twist)
		rospy.Subscriber('/haptic_base', geometry_msgs.msg.Twist, self.haptic_callback)
		
	def haptic_callback(self, msg):
		self.base_velocity.linear.x = msg.linear.x
		self.base_velocity.linear.y = msg.linear.y
		self.base_velocity.angular.z = msg.angular.z
		
		self.base_velocity_publisher.publish(self.base_velocity)
		
		
if __name__ == "__main__":
	rospy.init_node('haptic_client')
	haptic_client()
	rospy.loginfo('Haptic client running')
	rospy.spin()
