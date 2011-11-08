#! /usr/bin/env python
import roslib; roslib.load_manifest('youbot_android')
import rospy
import bluetooth

class youbot_android():	
	nearby_devices = bluetooth.discover_devices()
	target_name = 'Milestone XT720'
	target_address = None
	
	for bdaddr in nearby_devices:
		if target_name == bluetooth.lookup_name(bdaddr):
			target_address = bdaddr
			break
	
	if target_address is not None:
		print 'found target', target_address
	else:
		print 'could not found' 

if __name__ == '__main__':
	rospy.init_node('youbot_android')
	youbot_android()
	rospy.loginfo('youbot android node is running')
	rospy.spin()
