#!/usr/bin/env python

import rospy
from pr2_controllers_msgs.msg import Pr2GripperCommandAction, Pr2GripperCommandGoal
import actionlib


class GripperClient(object):
	
	def __init__(self,side=None):
		rospy.init_node('gripper_client')
		self._client = actionlib.SimpleActionClient('/' + side + '_gripper_controller/gripper_action', Pr2GripperCommandAction)
		self._goal = Pr2GripperCommandGoal()
		if not self._client.wait_for_server(rospy.Duration(10.0)):
			rospy.logerr('The gripper server is not connected')
			rospy.signal_shutdown("Action Server not found")
			import sys
			sys.exit(1)
		
	
	def command(self, position,effort):
		self._goal.command.position = position
		self._goal.command.max_effort = effort
		self._client.send_goal(self._goal)
	
	def stop(self):
		self._client.cancel_goal()
	
	def wait(self, timeout=10.0):
		self._client.wait_for_result(timeout =rospy.Duration(timeout))
		
		
	


if __name__ == '__main__':
	try:
		gripper = GripperClient(side='r')
		gripper.command(0.00, -1.0)
	except rospy.ROSInterruptException:
		pass
