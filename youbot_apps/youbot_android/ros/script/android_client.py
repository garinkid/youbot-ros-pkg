#! /usr/bin/env python
import roslib; roslib.load_manifest('youbot_android')
import rospy
import bluetooth
import 

class decript_command():

class base_movement():
	self.angular = 0.0
	self.transversal = 0.0
	self.longitudinal = 0.0	

class youbot_android():	
	def __init__:

	server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
	port = 1
	server_sock.bind(("",port))
	server_sock.listen(1)
	client_sock,address = server_sock.accept()
	print "Accepted connection from", address
	while True:
		data = client_sock.recv(1024)
		print "received %s" % data
	

if __name__ == '__main__':
	rospy.init_node('youbot_android')
	youbot_android()
	rospy.loginfo('youbot android node is running')
