#!/usr/bin/env python

import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_limits = {'min': [-1.5, -0.75],
                     'max': [1.5, 1.45]
                     }

key_mapping = {'\x1b': [0.0, -0.1],  # up tilt up
               '\x42': [0, 0.1],  # down tilt up
               '\x44': [0.1, 0],  # left pane left
               '\x43': [-0.1, 0],  # right pane tright
               '\x20': [0, 0]}
head_joint_positions = [0.0, 0.0]
print('a')
if __name__ == "__main__":
	rospy.init_node("head_motion")
	rospy.loginfo("Waiting for head_controller...")

	head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	head_client.wait_for_server()
	rospy.loginfo("...connected.")


	def keys_cb(msg):
		if len(msg.data) == 0 or not key_mapping.has_key(msg.data[0]):
			return  # unknown	key

		trajectory = JointTrajectory()
		trajectory.joint_names = head_joint_names
		trajectory.points.append(JointTrajectoryPoint())
		head_joint_positions[0] += key_mapping[msg.data[0]][0]
		head_joint_positions[0] = head_joint_limits['max'][0] if head_joint_positions[0] > head_joint_limits['max'][0] \
			else head_joint_positions[0]
		head_joint_positions[0] = head_joint_limits['min'][0] if head_joint_positions[0] < head_joint_limits['min'][0] \
			else head_joint_positions[0]

		head_joint_positions[1] += key_mapping[msg.data[0]][1]
		head_joint_positions[1] = head_joint_limits['max'][1] if head_joint_positions[1] > head_joint_limits['max'][
			1] else head_joint_positions[1]
		head_joint_positions[1] = head_joint_limits['min'][1] if head_joint_positions[1] < head_joint_limits['min'][
			1] else head_joint_positions[1]

		trajectory.points[0].positions = head_joint_positions
		trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
		trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
		trajectory.points[0].time_from_start = rospy.Duration(5)
		print(head_joint_positions)

		head_goal = FollowJointTrajectoryGoal()
		head_goal.trajectory = trajectory
		head_goal.goal_time_tolerance = rospy.Duration(0)

		head_client.send_goal(head_goal)
		head_client.wait_for_result(rospy.Duration(6))


	head_sub = rospy.Subscriber('keys', String, keys_cb)
	print('b')
	rospy.spin()
