#!/usr/bin/env python

import rospy, sys
import moveit_commander


class MoveItDemo:
	def __init__(self):
		rospy.init_node('moveit_demo_pr2', anonymous = True)
		# Initialize the move_group API
		# moveit_commander.roscpp_initialize(sys.argv)
		moveit_commander.roscpp_initialize(sys.argv)
		# Initialize the ROS node
		
		robot = moveit_commander.RobotCommander()
		
		
		
		rospy.sleep(2.0)
		#
		# 		GRIPPER_OPEN = [0.05]
		# 		GRIPPER_CLOSED = [-0.03]
		# 		GRIPPER_NEUTRAL = [0.01]
		#
		# left_arm = moveit_commander.MoveGroupCommander('left_arm')
		# joint_positions = [1.10, 0.37191, -1.49, 1.447, -1.178, -1.36, 1.99]
		# left_arm.set_joint_value_target(joint_positions)
		# #
		# # # Plan and execute the motion
		# left_arm.go()
		# rospy.sleep(1)

		#
		# 		# Connect to the left_gripper move group
		# pose = 0.02/1.0
		# GRIPPER_NEUTRAL =[pose,-pose]
		left_gripper = moveit_commander.MoveGroupCommander('left_gripper')
		# #
		# # 		# Get the name of the end-effector link
		# end_effector_link = left_arm.get_end_effector_link()
		# print('lo:    ',end_effector_link)
		#
		# 		# Display the name of the end_effector link
		# 		# rospy.loginfo("The end effector link is: " + str(end_effector_link))
		#
		# 		# Set a small tolerance on joint angles
		# 		left_arm.set_goal_joint_tolerance(0.001)
		# 		# left_gripper.set_goal_joint_tolerance(0.001)
		#
		# 		# Start the arm target in "resting" pose stored in the SRDF file
		# 		left_arm.set_named_target('left_neutral')
		#
		# 		# Plan a trajectory to the goal configuration
		# 		traj = left_arm.plan()
		#
		# 		# Execute the planned trajectory
		# 		left_arm.execute(traj)
		#
		# 		# Pause for a moment
		# 		rospy.sleep(1)
		#
		# 		# Set the gripper target to neutal position using a joint value target
		
		# a = left_gripper.get_active_joints()
		# b = left_gripper.get_joints()
		
		# left_gripper.set_joint_value_target([0.02,0.02])
		# left_gripper.go()
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# rospy.sleep(2.0)
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		left_gripper.set_random_target()
		# left_gripper.set_joint_value_target([0.02,-0.02])
		left_gripper.go()
		# left_gripper.go([0.02,-0.02])
		# rospy.sleep(2.0)
		#
		# left_gripper.go()
		# print(c)
	
		#
		# 		# Plan and execute the gripper motion
		# left_gripper.go(GRIPPER_NEUTRAL)
		# rospy.sleep(1)
		#
		# 		# Set target joint values for the arm: joints are in the order they appear in
		# 		# the kinematic tree.
		# 		#  # [-0.0867, -1.274, 0.02832, 0.0820, -1.273, -0.003]
		# 		#
		# 		# # Set the arm's goal configuration to the be the joint positions
	
		#
		# 		# Save this configuration for later
		# 		# left_arm.remember_joint_values('saved_config', joint_positions)
		#
		# 		# Close the gripper as if picking something up
		# 		# left_gripper.set_joint_value_target(GRIPPER_CLOSED)
		# 		# left_gripper.go()
		# 		# rospy.sleep(1)
		#
		# 		# Set the arm target to the named "straight_out" pose stored in the SRDF file
		# 		# left_arm.set_named_target('straight_forward')
		#
		# 		# Plan and execute the motion
		# 		# left_arm.go()
		# 		# rospy.sleep(1)
		# 		#
		# 		# Set the goal configuration to the named configuration saved earlier
		# 		# left_arm.set_named_target('saved_config')
		# 		#
		# 		# # Plan and execute the motion
		# 		# left_arm.go()
		# 		# rospy.sleep(1)
		#
		# 		# Open the gripper as if letting something go
		# 		# left_gripper.set_joint_value_target(GRIPPER_OPEN)
		# 		# left_gripper.go()
		# 		# rospy.sleep(1)
		# 		#
		# 		# Return the arm to the named "resting" pose stored in the SRDF file
		# 		# left_arm.set_named_target('resting')
		# 		# left_arm.go()
		# 		# rospy.sleep(1)
		#
		# 		# Return the gripper target to neutral position
		# 		# left_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
		# 		# left_gripper.go()
		# 		# rospy.sleep(1)
		#
		# 		# Cleanly shut down MoveIt
		moveit_commander.roscpp_shutdown()
		# #
		# # 		# Exit the script
		moveit_commander.os._exit(0)
		# rospy.spin()


if __name__ == "__main__":
	try:
		MoveItDemo()
	except rospy.ROSInterruptException:
		pass

# import argparse
# import sys
#
# from copy import copy
#
# import rospy
#
# import actionlib
#
# from control_msgs.msg import (
# 	FollowJointTrajectoryAction,
# 	FollowJointTrajectoryGoal,
# )
# from trajectory_msgs.msg import (
# 	JointTrajectoryPoint,
# )
#
# import baxter_interface
#
# from baxter_interface import CHECK_VERSION
#
#
# class Trajectory(object):
# 	def __init__(self, limb):
# 		ns = 'robot/limb/' + limb + '/'
# 		self._client = actionlib.SimpleActionClient(
# 			ns + "follow_joint_trajectory",
# 			FollowJointTrajectoryAction,
# 		)
# 		self._goal = FollowJointTrajectoryGoal()
# 		self._goal_time_tolerance = rospy.Time(0.1)
# 		self._goal.goal_time_tolerance = self._goal_time_tolerance
# 		server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
# 		if not server_up:
# 			rospy.logerr("Timed out waiting for Joint Trajectory"
# 						 " Action Server to connect. Start the action server"
# 						 " before running example.")
# 			rospy.signal_shutdown("Timed out waiting for Action Server")
# 			sys.exit(1)
# 		self.clear(limb)
#
# 	def add_point(self, positions, time):
# 		point = JointTrajectoryPoint()
# 		point.positions = copy(positions)
# 		point.time_from_start = rospy.Duration(time)
# 		self._goal.trajectory.points.append(point)
#
# 	def start(self):
# 		self._goal.trajectory.header.stamp = rospy.Time.now()
# 		self._client.send_goal(self._goal)
#
# 	def stop(self):
# 		self._client.cancel_goal()
#
# 	def wait(self, timeout=15.0):
# 		self._client.wait_for_result(timeout=rospy.Duration(timeout))
#
# 	def result(self):
# 		return self._client.get_result()
#
# 	def clear(self, limb):
# 		self._goal = FollowJointTrajectoryGoal()
# 		self._goal.goal_time_tolerance = self._goal_time_tolerance
# 		self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
# 			['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
#
#
# def main():
# 	"""RSDK Joint Trajectory Example: Simple Action Client
#
# 	Creates a client of the Joint Trajectory Action Server
# 	to send commands of standard action type,
# 	control_msgs/FollowJointTrajectoryAction.
#
# 	Make sure to start the joint_trajectory_action_server.py
# 	first. Then run this example on a specified limb to
# 	command a short series of trajectory points for the arm
# 	to follow.
# 	"""
# 	arg_fmt = argparse.RawDescriptionHelpFormatter
# 	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
# 									 description=main.__doc__)
# 	required = parser.add_argument_group('required arguments')
# 	required.add_argument(
# 		'-l', '--limb', required=True, choices=['left', 'right'],
# 		help='send joint trajectory to which limb'
# 	)
# 	args = parser.parse_args(rospy.myargv()[1:])
# 	limb = args.limb
#
# 	print("Initializing node... ")
# 	rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
# 	print("Getting robot state... ")
# 	rs = baxter_interface.RobotEnable(CHECK_VERSION)
# 	print("Enabling robot... ")
# 	rs.enable()
# 	print("Running. Ctrl-c to quit")
# 	positions = {
# 	    'left':  [ 1.10, 0.37191,-1.49, 1.447, -1.178, -1.36, 1.99], #[-0.11, -0.62, -1.15, 1.32,  0.80, 1.27,  2.39],
# 	    'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
# 	}
#
# 	traj = Trajectory(limb)
# 	# rospy.on_shutdown(traj.stop)
# 	# Command Current Joint Positions first
# 	limb_interface = baxter_interface.limb.Limb(limb)
# 	current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
# 	traj.add_point(current_angles, 0.0)
# 	p1 = positions[limb]
# 	traj.add_point(p1, 30.0)
# 	# traj.add_point([x * 0.75 for x in p1], 9.0)
# 	# traj.add_point([x * 1.25 for x in p1], 12.0)
# 	traj.start()
# 	traj.wait(100.0)
# 	print("Exiting - Joint Trajectory Action Test Complete")
#
# if __name__ == "__main__":
# 	main()
