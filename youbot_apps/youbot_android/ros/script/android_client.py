#! /usr/bin/env python
import roslib; roslib.load_manifest('youbot_android')
import rospy
import bluetooth
import math
import sys
import threading
from brics_actuator.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *	
from struct import unpack

class Bluetooth_Thread(threading.Thread):
	def run(self):
		self.server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
		self.port = 1
		self.command = ''
		self.data = []


		self.server_sock.bind(("",self.port))
		self.server_sock.listen(1)

		unit = 's^-1 rad' #'rad'
		self.maximum_velocity = 0.5
		self.command_velocity = brics_actuator.msg.JointVelocities()
		poison_stamp = brics_actuator.msg.Poison()
		poison_stamp.originator = ""
		self.joint_value_1 = brics_actuator.msg.JointValue()
		self.joint_value_1.joint_uri = 'arm_joint_1'
		self.joint_value_1.unit = unit
		self.joint_value_1.value = 0.0
		self.joint_value_2 = brics_actuator.msg.JointValue()
		self.joint_value_2.joint_uri = 'arm_joint_2'
		self.joint_value_2.unit = unit
		self.joint_value_2.value = 0.0
		self.joint_value_3 = brics_actuator.msg.JointValue()
		self.joint_value_3.joint_uri = 'arm_joint_3'
		self.joint_value_3.unit = unit
		self.joint_value_3.value = 0.0
		self.joint_value_4 = brics_actuator.msg.JointValue()
		self.joint_value_4.joint_uri = 'arm_joint_4'
		self.joint_value_4.unit = unit
		self.joint_value_4.value = 0.0
		self.joint_value_5 = brics_actuator.msg.JointValue()
		self.joint_value_5.joint_uri = 'arm_joint_5'
		self.joint_value_5.unit = unit
		self.joint_value_5.value = 0.0
		self.velocities = [self.joint_value_1, self.joint_value_2, self.joint_value_3, self.joint_value_4, self.joint_value_5]
		self.command_velocity.poisonStamp = poison_stamp
		self.command_velocity.velocities = self.velocities

		self.base_velocity = geometry_msgs.msg.Twist()

		self.position_command = brics_actuator.msg.JointPositions()
		self.unit_position = 'rad'# 's^-1 rad' #
		self.poison_stamp = brics_actuator.msg.Poison()
		self.poison_stamp.originator = ""
		self.joint_value_1_position = brics_actuator.msg.JointValue()
		self.joint_value_1_position.joint_uri = 'arm_joint_1'
		self.joint_value_1_position.unit = self.unit_position
		self.joint_value_1_position.value = 0.0
		self.joint_value_2_position = brics_actuator.msg.JointValue()
		self.joint_value_2_position.joint_uri = 'arm_joint_2'
		self.joint_value_2_position.unit = self.unit_position
		self.joint_value_2_position.value = 0.0
		self.joint_value_3_position = brics_actuator.msg.JointValue()
		self.joint_value_3_position.joint_uri = 'arm_joint_3'
		self.joint_value_3_position.unit = self.unit_position
		self.joint_value_3_position.value = 0.0
		self.joint_value_4_position = brics_actuator.msg.JointValue()
		self.joint_value_4_position.joint_uri = 'arm_joint_4'
		self.joint_value_4_position.unit = self.unit_position
		self.joint_value_4_position.value = 0.0
		self.joint_value_5_position = brics_actuator.msg.JointValue()
		self.joint_value_5_position.joint_uri = 'arm_joint_5'
		self.joint_value_5_position.unit = self.unit_position
		self.joint_value_5_position.value = 0.0
		self.positions = [self.joint_value_1_position, self.joint_value_2_position, self.joint_value_3_position, self.joint_value_4_position, self.joint_value_5_position]
		self.position_command.poisonStamp = self.poison_stamp
		self.position_command.positions = self.positions

		self.data_format = True

		print 'Waiting for connection...'
		self.client_sock, self.address = self.server_sock.accept()
		print "Accepted connection from", self.address
		while not rospy.is_shutdown():
			self.command = self.client_sock.recv(1024)
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
				self.data_format = True
				for i in range((len(self.data)) - 1):
					try:
						float(self.data[i+1])
					except ValueError:
						self.data_format = False
				if self.data_format is True:
					if self.data[0] == 'base':
						self.android_base(self.data)
					elif self.data[0] == 'manipulator': 
						self.android_manipulator(self.data)
					elif self.data[0] == 'arm_joint_position': 
						self.android_arm_joint_position(self.data)	
		
	def android_base(self, data):
		for i in range(3):
			data[i+1] = float(self.data[i+1]) 
		max_translational_velocity_x = 0.25 # m/s
		max_translational_velocity_y = 0.25 # m/s
		max_angular_velocity_z = 0.4 # rad/s
		self.base_velocity.linear.x = data[1] *  max_translational_velocity_x
		self.base_velocity.linear.y = data[2] * max_translational_velocity_y 
		self.base_velocity.angular.z = data[3]  * max_angular_velocity_z
		print 'base velocity --> linear x:', round(self.base_velocity.linear.x, 2), 'm/s, linear y:',  round(self.base_velocity.linear.y, 2), 'm/s, angular z',  round(self.base_velocity.angular.z, 2), 'rad/s'

	def android_manipulator(self, data):
		maximum_velocity = 1.00 # radian / second
		self.joint_value_1.value = float(data[1]) * maximum_velocity
		self.joint_value_2.value = float(data[2]) * maximum_velocity
		self.joint_value_3.value = float(data[3]) * maximum_velocity
		self.joint_value_4.value = float(data[4]) * maximum_velocity
		self.joint_value_5.value = float(data[5]) * maximum_velocity
		self.velocities = [self.joint_value_1, self.joint_value_2, self.joint_value_3, self.joint_value_4, self.joint_value_5]
		self.command_velocity.velocities = self.velocities
		print 'velocity set(rad/s) --> joint 1: '+ str(round(self.joint_value_1.value, 2)) +',', 'joint 2: ' +str(round(self.joint_value_2.value, 2)) + ',', 'joint 3: ' + str(round(self.joint_value_3.value, 2)) + ',', 'joint 4: ' +  str(round(self.joint_value_4.value, 2)) + ',' , 'joint 5: ' + str(round(self.joint_value_5.value, 2)) 

	def android_arm_joint_position(self, data):
  		# blender reference offset = [169, 155, -142, 168, 171]
		print 'set position(degree) --> joint 1:' + data[1] + ',' , 'joint 2:' + data[2] + ',',  'joint 3:' + data[3] + ',',  'joint 4:' + data[4] + ',', 'joint 5:' + data[5]

 		offset = [169, 65, -142, 108, 171]
		for i in range(5):
			data[i+1] = math.radians(float(data[i+1]) + offset[i])
		min_range = [0.0100693, 0.0100693, -5.02656, 0.0221240, 0.110620]
		max_range = [5.84013, 2.61798, -0.015709, 3.4291, 5.64158]
		for j in range(5):
			if data[j+1] < min_range[j]:
				data[j+1] = min_range[j]
			elif data[j+1] > max_range[j]:
				data[j+1] = max_range[j]	
		self.joint_value_1_position.value = data[1]
		self.joint_value_2_position.value = data[2]
		self.joint_value_3_position.value = data[3]
		self.joint_value_4_position.value = data[4]
		self.joint_value_5_position.value = data[5]
		self.positions = [self.joint_value_1_position, self.joint_value_2_position, self.joint_value_3_position, self.joint_value_4_position, self.joint_value_5_position]
		self.position_command.positions = self.positions


class youbot_android():	
	def __init__(self):
		rospy.loginfo('youbot android node is running')
		bluetooth_thread = Bluetooth_Thread()
		bluetooth_thread.setDaemon(True)
		bluetooth_thread.start()

		# check parameter		
		if rospy.has_param(str(self.__class__.__name__) +'/simulation'):
			self.simulation = rospy.get_param(str(self.__class__.__name__) +'/simulation')
		else:
			print (str(self.__class__.__name__) +'/simulation parameter is set to default')
			self.simulation = True
		print 'Simulation:', self.simulation

		if rospy.has_param(str(self.__class__.__name__) +'/soft_stop_threshold'):
			self.soft_stop_threshold = rospy.get_param(str(self.__class__.__name__) +'/soft_stop_threshold')
		else:
			print  (str(self.__class__.__name__) +'/soft_stop_threshold parameter is set to default')
			self.soft_stop_threshold = 0.5
		print 'Soft stop threshold:', self.soft_stop_threshold, ' rad'		

		self.base_velocity = geometry_msgs.msg.Twist()
		self.joint_velocity =  brics_actuator.msg.JointVelocities()
		self.joint_position = brics_actuator.msg.JointPositions()
		self.base_velocity_publisher = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist)
		self.joint_velocity_publisher = rospy.Publisher('/arm_1/arm_controller/velocity_command', brics_actuator.msg.JointVelocities)
		self.joint_position_publisher = rospy.Publisher('/arm_1/arm_controller/position_command', brics_actuator.msg.JointPositions)	

		self.min_joint_limit = [False] * 5
		self.max_joint_limit = [False] * 5

		if self.simulation is True:
			rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.state_callback)
		elif self.simulation is False:
			rospy.Subscriber('/arm_1/joint_states', sensor_msgs.msg.JointState, self.state_callback)
		else:
			print "Simulation parameter is set to default 'True' "
			self.simulation = True
			rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self.state_callback)

		self.base_velocity = bluetooth_thread.base_velocity	
		self.joint_position = bluetooth_thread.position_command
		self.joint_velocity = bluetooth_thread.command_velocity
		while not rospy.is_shutdown():
			if len(bluetooth_thread.data) > 0:
				if bluetooth_thread.data[0] == 'base':
					self.base_velocity = bluetooth_thread.base_velocity	
				elif bluetooth_thread.data[0] == 'arm_joint_position':
					self.joint_position = bluetooth_thread.position_command
				elif bluetooth_thread.data[0] == 'manipulator':
					self.joint_velocity = bluetooth_thread.command_velocity	
			for i in range(5):
				if (self.min_joint_limit[i] is True) and (self.joint_velocity.velocities[i].value < 0.0) :
					print 'joint', (i + 1),  'joint limit reached, joint', (i+1),  'velocity set to 0.0 rad/s'
					self.joint_velocity.velocities[i].value = 0.0
				elif (self.max_joint_limit[i] is True) and (self.joint_velocity.velocities[i].value > 0.0) :
					print 'joint', (i + 1),   'joint limit reached, joint', (i+1),  'velocity set to 0.0 rad/s'
					self.joint_velocity.velocities[i].value = 0.0
			self.base_velocity_publisher.publish(self.base_velocity)	
			self.joint_position_publisher.publish(self.joint_position)
			self.joint_velocity_publisher.publish(self.joint_velocity)	
			
	
	def state_callback(self, msg):
		# actual_limit
		min_range = [0.0100693, 0.0100693, -5.02656, 0.0221240, 0.110620]
		max_range = [5.84013, 2.61798, -0.015709, 3.4291, 5.64158]	
		# soft_limit
		for i in range(5):
			min_range[i] = min_range[i] + self.soft_stop_threshold
			max_range[i] = max_range[i] - self.soft_stop_threshold 
		current_position = [0.0] * 5

		for i in range(5):
			if self.simulation is True:
				current_position[i] = msg.position[i+8]
			else:
				current_position[i] = msg.position[i]

		for i in range(5):
			if current_position[i] < min_range[i]:
				self.min_joint_limit[i] = True
				self.max_joint_limit[i] = False
			elif current_position[i] > max_range[i]:
				self.max_joint_limit[i] = True
				self.min_joint_limit[i] = False
			else:
				self.min_joint_limit[i] = False
				self.max_joint_limit[i] = False				

if __name__ == '__main__':
	rospy.init_node('youbot_android')
	youbot_android()
	rospy.spin()
