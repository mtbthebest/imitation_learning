#!/usr/bin/env python

import rospy, sys
import moveit_commander
import rbt_baxter_interface


class MoveItDemo:
	def __init__(self):
		rospy.init_node('moveit_demo_baxter', anonymous = True)
		# Initialize the move_group API
		# moveit_commander.roscpp_initialize(sys.argv)
		moveit_commander.roscpp_initialize('/joint_states:=/robot/joint_states')
		# Initialize the ROS node

		robot = moveit_commander.RobotCommander()

		rbt_baxter_interface.RobotEnable().enable()

		rospy.sleep(2.0)
		#
		# 		GRIPPER_OPEN = [0.05]
		# 		GRIPPER_CLOSED = [-0.03]
		# 		GRIPPER_NEUTRAL = [0.01]
		#

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
		left_arm = moveit_commander.MoveGroupCommander('left_arm')
		left_gripper = moveit_commander.MoveGroupCommander('left_hand')
		# #
		# # 		# Get the name of the end-effector link
		end_effector_link = left_arm.get_end_effector_link()
		
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

		a = left_gripper.get_active_joints()
		b = left_gripper.get_joints()
		print(a,b)

		left_gripper.set_joint_value_target([0.02,0.0])
		left_gripper.go()
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# rospy.sleep(2.0)
		# left_gripper.set_joint_value_target([0.02, -0.02])
		# left_gripper.go()
		# c = left_gripper.get_current_joint_values()
		# left_gripper.set_joint_value_target([0.02,-0.02])
		# left_gripper.go()
		# left_gripper.go([0.02,-0.02])
		# rospy.sleep(2.0)
		#
		# left_gripper.go()
		# print(a, b, c)

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