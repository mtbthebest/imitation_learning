#!/usr/bin/env python

# Copyright (c) 2015, Fetch Robotics Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Fetch Robotics Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL FETCH ROBOTICS INC. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Author: Michael Ferguson

import sys

import rospy
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

arm_controllers = ['right', 'left']
arm_joint_names = ["s0", "s1", "e0", "e1", "w0", "w1", "w2"]
arm_intermediate_positions = [0.0] * len(arm_joint_names)
right_arm_joint_positions = [0.75, -0.54, 0.0, -0.05, 0.0, 1.41, 0.0]
left_arm_joint_positions = [0.0, 0.4, -0.5, 0.5, 0, 0, 1.3]

head_joint_names = ["head_pan_joint", "head_tilt_joint"]
head_joint_positions = [0.0, 0.0]

if __name__ == "__main__":
	rospy.init_node("prepare_simulated_robot")
	# Check robot serial number, we never want to run this on a real robot!
	# if rospy.get_param("robot/serial") != "ABCDEFGHIJKLMNOPQRSTUVWX":
	# 	rospy.logerr("This script should not be run on a real robot")
	# 	sys.exit(-1)
	
	# rospy.loginfo("Waiting for head_controller...")
	# head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
	# head_client.wait_for_server()
	# rospy.loginfo("...connected.")
	
	for controllers in arm_controllers[1:]:
		duration = 10
		rospy.loginfo("Waiting for arm_controller...")
		arm_client = actionlib.SimpleActionClient(controllers + '_arm_controller/follow_joint_trajectory',
		                                          FollowJointTrajectoryAction)
		arm_client.wait_for_server()
		rospy.loginfo("...connected.")
		
		arm_joint_positions = right_arm_joint_positions if controllers == 'right' else left_arm_joint_positions
		trajectory = JointTrajectory()
		trajectory.joint_names = [controllers + '_' + joint_names for joint_names in arm_joint_names]
		trajectory.points.append(JointTrajectoryPoint())
		trajectory.points[0].positions = [0.0] * len(arm_joint_positions)
		trajectory.points[0].velocities = [0.0] * len(arm_joint_positions)
		trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
		trajectory.points[0].time_from_start = rospy.Duration(duration)
		# duration +=5
		# trajectory.points.append(JointTrajectoryPoint())
		# trajectory.points[1].positions =  [0.0] * len(arm_joint_positions)
		# trajectory.points[1].velocities = [0.0] * len(arm_joint_positions)
		# trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
		# trajectory.points[1].time_from_start = rospy.Duration(duration)
		# duration +=5
		# trajectory.points.append(JointTrajectoryPoint())
		# trajectory.points[2].positions = arm_joint_positions
		# trajectory.points[2].velocities = [0.0] * len(arm_joint_positions)
		# trajectory.points[2].accelerations = [0.0] * len(arm_joint_positions)
		# trajectory.points[2].time_from_start = rospy.Duration(duration)
		
		arm_goal = FollowJointTrajectoryGoal()
		arm_goal.trajectory = trajectory
		
		# gripper_goal = GripperCommandGoal()
		# gripper_goal.command.max_effort = 10.0
		# gripper_goal.command.position = 0.1
		
		rospy.loginfo("Setting positions...")
		# head_client.send_goal(head_goal)
		arm_client.send_goal(arm_goal)
		# gripper_client.send_goal(gripper_goal)
		# gripper_client.wait_for_result(rospy.Duration(5))
		arm_client.wait_for_result(rospy.Duration(100))
		# rospy.sleep(5)
		# head_client.wait_for_result(rospy.Duration(6))
rospy.loginfo("...done")
