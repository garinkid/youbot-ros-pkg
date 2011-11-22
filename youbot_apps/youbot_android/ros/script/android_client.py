#! /usr/bin/env python
import roslib; roslib.load_manifest('youbot_android')
import rospy
import bluetooth
import sys
import threading
from struct import unpack
from geometry_msgs.msg import *

class Bluetooth_Thread(threading.Thread):
	def run(self):
		self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		self.port = 1
		self.command = ''
		self.data = []

		self.base_velocity = geometry_msgs.msg.Twist()
		self.base_velocity.linear.x = 0.0
		self.base_velocity.linear.y = 0.0
		self.base_velocity.linear.z = 0.0
		self.base_velocity.angular.x = 0.0
		self.base_velocity.angular.y = 0.0
		self.base_velocity.angular.z = 0.0

		self.normalization_linear_x = 0.25
		self.normalization_linear_y = 0.25
		self.normalization_angular_z = 0.5
		
		self.server_sock.bind(("",self.port))
		self.server_sock.listen(1)

		print 'Waiting for connection...'
		self.client_sock, self.address = self.server_sock.accept()
		print "Accepted connection from", self.address
		while not rospy.is_shutdown():
			self.command = self.client_sock.recv(1024)
			print "received %s" % self.command
			if self.command == 'onPause':
				self.client_sock.close()
				self.server_sock.close()
				self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
				self.server_sock.bind(("",self.port))
				self.server_sock.listen(1)
				print 'Android paused, waiting for connection...'
				self.client_sock, self.address = self.server_sock.accept()
				print "Accepted connection from", self.address
			else:
				self.data = self.command.split(',')
				# data will be received in range 0 - 100
				for i in range(len(self.data)):
					self.data[i] = float(self.data[i]) * 0.01
				self.base_velocity.linear.x = self.data[0] * self.normalization_linear_x
				self.base_velocity.linear.y = self.data[1] * self.normalization_linear_y
				self.base_velocity.angular.z = self.data[2] * self.normalization_angular_z


class youbot_android():	
	def __init__(self):
		rospy.loginfo('youbot android node is running')
		
		bluetooth_thread = Bluetooth_Thread()
		bluetooth_thread.setDaemon(True)
		bluetooth_thread.start()

		self.base_velocity = geometry_msgs.msg.Twist()
		self.base_velocity_publisher = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)
		while not rospy.is_shutdown():
			self.base_velocity = bluetooth_thread.base_velocity
			self.base_velocity_publisher.publish(self.base_velocity)		

if __name__ == '__main__':
	rospy.init_node('youbot_android')
	youbot_android()
	rospy.spin()
