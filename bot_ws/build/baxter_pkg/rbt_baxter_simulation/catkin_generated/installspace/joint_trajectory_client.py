#!/usr/bin/env python

import argparse
import sys

from copy import copy

import rospy

import actionlib

from control_msgs.msg import (
	FollowJointTrajectoryAction,
	FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
	JointTrajectoryPoint,
)

import rbt_baxter_interface

from rbt_baxter_interface import CHECK_VERSION


class Trajectory(object):
	def __init__(self, limb):
		ns = 'robot/limb/' + limb + '/'
		self._client = actionlib.SimpleActionClient(
			ns + "follow_joint_trajectory",
			FollowJointTrajectoryAction,
		)
		self._goal = FollowJointTrajectoryGoal()
		self._goal_time_tolerance = rospy.Time(0.1)
		self._goal.goal_time_tolerance = self._goal_time_tolerance
		server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
		if not server_up:
			rospy.logerr("Timed out waiting for Joint Trajectory"
						 " Action Server to connect. Start the action server"
						 " before running example.")
			rospy.signal_shutdown("Timed out waiting for Action Server")
			sys.exit(1)
		self.clear(limb)

	def add_point(self, positions, time):
		point = JointTrajectoryPoint()
		point.positions = copy(positions)
		point.time_from_start = rospy.Duration(time)
		self._goal.trajectory.points.append(point)

	def start(self):
		self._goal.trajectory.header.stamp = rospy.Time.now()
		self._client.send_goal(self._goal)

	def stop(self):
		self._client.cancel_goal()

	def wait(self, timeout=15.0):
		self._client.wait_for_result(timeout=rospy.Duration(timeout))

	def result(self):
		return self._client.get_result()

	def clear(self, limb):
		self._goal = FollowJointTrajectoryGoal()
		self._goal.goal_time_tolerance = self._goal_time_tolerance
		self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
			['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def main():
	"""RSDK Joint Trajectory Example: Simple Action Client

	Creates a client of the Joint Trajectory Action Server
	to send commands of standard action type,
	control_msgs/FollowJointTrajectoryAction.

	Make sure to start the joint_trajectory_action_server.py
	first. Then run this example on a specified limb to
	command a short series of trajectory points for the arm
	to follow.
	"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									 description=main.__doc__)
	required = parser.add_argument_group('required arguments')
	required.add_argument(
		'-l', '--limb', required=True, choices=['left', 'right'],
		help='send joint trajectory to which limb'
	)
	args = parser.parse_args(rospy.myargv()[1:])
	limb = args.limb

	print("Initializing node... ")
	rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
	print("Getting robot state... ")
	rs = rbt_baxter_interface.RobotEnable(CHECK_VERSION)
	print("Enabling robot... ")
	rs.enable()
	print("Running. Ctrl-c to quit")
	positions = {
	    'left': [-0.11, -0.62, -1.15, 1.32, 0.80, 1.27, 2.39],#[1.10, 0.37191,-1.49, 1.447, -1.178, -1.36, 1.99], #,
	    'right':  [0.11, -0.62,  1.15, 1.32, -0.80, 1.27, -2.39],
	}

	traj = Trajectory(limb)
	# rospy.on_shutdown(traj.stop)
	# Command Current Joint Positions first
	limb_interface = rbt_baxter_interface.limb.Limb(limb)
	current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
	traj.add_point(current_angles, 0.0)
	p1 = positions[limb]
	traj.add_point(p1, 10.0)
	# traj.add_point([x * 0.75 for x in p1], 9.0)
	# traj.add_point([x * 1.25 for x in p1], 12.0)
	traj.start()
	traj.wait(100.0)
	print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
	main()