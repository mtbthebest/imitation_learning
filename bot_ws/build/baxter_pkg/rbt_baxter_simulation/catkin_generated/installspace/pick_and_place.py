#!/usr/bin/env python
import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose, Point
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import (
	FollowJointTrajectoryAction,
	FollowJointTrajectoryGoal,
)
import actionlib
from tf.transformations import quaternion_from_euler
from copy import deepcopy, copy

import rbt_baxter_interface

from rbt_baxter_interface import CHECK_VERSION
import random
from math import sqrt
from collections import OrderedDict
import rospkg
from gazebo_msgs.srv import (
	SpawnModel,
	DeleteModel,
)
from gazebo_msgs.msg import ModelStates

GROUP_NAME_ARM = 'left_arm'
GROUP_NAME_GRIPPER = 'left_gripper'

GRIPPER_FRAME = 'left_gripper_link'

GRIPPER_OPEN = [0.03]
GRIPPER_CLOSED = [-0.02]
GRIPPER_NEUTRAL = [0.01]

GRIPPER_JOINT_NAMES = ['right_gripper_finger_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world'

model_size ={ 'xaf'






}


class PickAndPlace(object):
	def __init__(self, limb='left'):
		
		self.gazebo_client = GazeboClient(skip_models = ['baxter', 'ground_plane'])
		# Use the planning scene object to add or remove objects
		self.scene = PlanningSceneInterface()
		
		# Create a scene publisher to push changes to the scene
		self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size = 5)
		
		# Create a publisher for displaying gripper poses
		# self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
		
		# Create a dictionary to hold object colors
		self.colors = dict()
		
		# Initialize the move group for the right arm
		# self.arm = MoveGroupCommander(GROUP_NAME_ARM)
		#
		# # Initialize the move group for the right gripper
		# self.gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
		#
		# # Get the name of the end-effector link
		# self.end_effector_link = self.arm.get_end_effector_link()
		#
		# # Allow some leeway in position (meters) and orientation (radians)
		# self.arm.set_goal_position_tolerance(0.05)
		# self.arm.set_goal_orientation_tolerance(0.1)
		#
		# # Allow replanning to increase the odds of a solution
		# self.arm.allow_replanning(True)
		#
		# # Set the right arm reference frame
		# self.arm.set_pose_reference_frame(REFERENCE_FRAME)
		#
		# # Allow 5 seconds per planning attempt
		# self.arm.set_planning_time(5)
		#
		# # Set a limit on the number of pick attempts before bailing
		# self.max_pick_attempts = 5
		#
		# Set a limit on the number of place attempts
		# self.max_place_attempts = 5
		
		# Give the scene a chance to catch up
		# rospy.sleep(2)
	
	def get_scene(self):
		try:
			self.get_objects()
		except:
			self.objects = dict()
			self.get_objects()
	
	def get_objects(self):
		model_states = self.gazebo_client.get_model_states()
		for id in model_states.name:
			if id not in self.gazebo_client.skip_models:
				try:
					self.objects[id]['id'] = id
				except KeyError:
					self.objects[id] = dict()
					self.objects[id]['id'] = id
					self.objects[id]['pose'] = model_states.pose[model_states.name.index(id)]
				
				# if 'size' not in self.objects[id].keys():
				# 	self.objects[id]['pose']
		print(self.objects)
		
			
			
		
		# Give each of the scene objects a unique name
		# table_id = 'table'
		# box1_id = 'box1'
		# box2_id = 'box2'
		# target_id = 'target'
		# tool_id = 'tool'
		#
		# # Remove leftover objects from a previous run
		# scene.remove_world_object(table_id)
		# scene.remove_world_object(box1_id)
		# scene.remove_world_object(box2_id)
		# scene.remove_world_object(target_id)
		# scene.remove_world_object(tool_id)
		#
		# # Remove any attached objects from a previous session
		# scene.remove_attached_object(GRIPPER_FRAME, target_id)
		#
		# # Give the scene a chance to catch up
		# rospy.sleep(1)
		#
		# # Start the arm in the "resting" pose stored in the SRDF file
		# self.arm.set_named_target('resting')
		# self.arm.go()
		#
		# # Open the gripper to the neutral position
		# gripper.set_joint_value_target(GRIPPER_NEUTRAL)
		# gripper.go()
		#
		# rospy.sleep(1)
		#
		# # Set the height of the table off the ground
		# table_ground = 0.75
		#
		# # Set the dimensions of the scene objects [l, w, h]
		# table_size = [0.2, 0.7, 0.01]
		# box1_size = [0.1, 0.05, 0.05]
		# box2_size = [0.05, 0.05, 0.15]
		#
		# # Set the target size [l, w, h]
		# target_size = [0.02, 0.01, 0.12]
		#
		# # Add a table top and two boxes to the scene
		# table_pose = PoseStamped()
		# table_pose.header.frame_id = REFERENCE_FRAME
		# table_pose.pose.position.x = 0.25
		# table_pose.pose.position.y = 0.0
		# table_pose.pose.position.z = table_ground + table_size[2] / 2.0
		# table_pose.pose.orientation.w = 1.0
		# scene.add_box(table_id, table_pose, table_size)
		#
		# box1_pose = PoseStamped()
		# box1_pose.header.frame_id = REFERENCE_FRAME
		# box1_pose.pose.position.x = 0.21
		# box1_pose.pose.position.y = -0.1
		# box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
		# box1_pose.pose.orientation.w = 1.0
		# scene.add_box(box1_id, box1_pose, box1_size)
		#
		# box2_pose = PoseStamped()
		# box2_pose.header.frame_id = REFERENCE_FRAME
		# box2_pose.pose.position.x = 0.19
		# box2_pose.pose.position.y = 0.13
		# box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
		# box2_pose.pose.orientation.w = 1.0
		# scene.add_box(box2_id, box2_pose, box2_size)
		#
		# # Set the target pose in between the boxes and on the table
		# target_pose = PoseStamped()
		# target_pose.header.frame_id = REFERENCE_FRAME
		# target_pose.pose.position.x = 0.22
		# target_pose.pose.position.y = 0.0
		# target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
		# target_pose.pose.orientation.w = 1.0
		#
		# # Add the target object to the scene
		# scene.add_box(target_id, target_pose, target_size)
		#
		# # Make the table red and the boxes orange
		# self.setColor(table_id, 0.8, 0, 0, 1.0)
		# self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
		# self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
		#
		# # Make the target yellow
		# self.setColor(target_id, 0.9, 0.9, 0, 1.0)
		#
		# # Send the colors to the planning scene
		# self.sendColors()
		#
		# # Set the support surface name to the table object
		# self.arm.set_support_surface_name(table_id)
		#
		# # Specify a pose to place the target after being picked up
		# place_pose = PoseStamped()
		# place_pose.header.frame_id = REFERENCE_FRAME
		# place_pose.pose.position.x = 0.18
		# place_pose.pose.position.y = -0.18
		# place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
		# place_pose.pose.orientation.w = 1.0
		#
		# # Initialize the grasp pose to the target pose
		# grasp_pose = target_pose
		#
		# # Shift the grasp pose by half the width of the target to center it
		# grasp_pose.pose.position.y -= target_size[1] / 2.0
		#
		# # Generate a list of grasps
		# grasps = self.make_grasps(grasp_pose, [target_id])
		#
		# # Publish the grasp poses so they can be viewed in RViz
		# for grasp in grasps:
		# 	self.gripper_pose_pub.publish(grasp.grasp_pose)
		# 	rospy.sleep(0.2)
		#
		# # Track success/failure and number of attempts for pick operation
		# result = None
		# n_attempts = 0
		#
		# # Repeat until we succeed or run out of attempts
		# while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
		# 	n_attempts += 1
		# 	rospy.loginfo("Pick attempt: " + str(n_attempts))
		# 	result = self.arm.pick(target_id, grasps)
		# 	rospy.sleep(0.2)
		#
		# # If the pick was successful, attempt the place operation
		# if result == MoveItErrorCodes.SUCCESS:
		# 	result = None
		# 	n_attempts = 0
		#
		# 	# Generate valid place poses
		# 	places = self.make_places(place_pose)
		#
		# 	# Repeat until we succeed or run out of attempts
		# 	while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
		# 		n_attempts += 1
		# 		rospy.loginfo("Place attempt: " + str(n_attempts))
		# 		for place in places:
		# 			result = self.arm.place(target_id, place)
		# 			if result == MoveItErrorCodes.SUCCESS:
		# 				break
		# 		rospy.sleep(0.2)
		#
		# 	if result != MoveItErrorCodes.SUCCESS:
		# 		rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
		# else:
		# 	rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
		#
		# # Return the arm to the "resting" pose stored in the SRDF file
		# self.arm.set_named_target('resting')
		# self.arm.go()
		#
		# # Open the gripper to the neutral position
		# gripper.set_joint_value_target(GRIPPER_NEUTRAL)
		# gripper.go()
		#
		# rospy.sleep(1)
		#
		# # Shut down MoveIt cleanly
		# moveit_commander.roscpp_shutdown()
		#
		# # Exit the script
		# moveit_commander.os._exit(0)
	
	# Get the gripper posture as a JointTrajectory
	def make_gripper_posture(self, joint_positions):
		# Initialize the joint trajectory for the gripper joints
		t = JointTrajectory()
		
		# Set the joint names to the gripper joint names
		t.joint_names = GRIPPER_JOINT_NAMES
		
		# Initialize a joint trajectory point to represent the goal
		tp = JointTrajectoryPoint()
		
		# Assign the trajectory joint positions to the input positions
		tp.positions = joint_positions
		
		# Set the gripper effort
		tp.effort = GRIPPER_EFFORT
		
		tp.time_from_start = rospy.Duration(1.0)
		
		# Append the goal point to the trajectory points
		t.points.append(tp)
		
		# Return the joint trajectory
		return t
	
	# Generate a gripper translation in the direction given by vector
	def make_gripper_translation(self, min_dist, desired, vector):
		# Initialize the gripper translation object
		g = GripperTranslation()
		
		# Set the direction vector components to the input
		g.direction.vector.x = vector[0]
		g.direction.vector.y = vector[1]
		g.direction.vector.z = vector[2]
		
		# The vector is relative to the gripper frame
		g.direction.header.frame_id = GRIPPER_FRAME
		
		# Assign the min and desired distances from the input
		g.min_distance = min_dist
		g.desired_distance = desired
		
		return g
	
	# Generate a list of possible grasps
	def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
		# Initialize the grasp object
		g = Grasp()
		
		# Set the pre-grasp and grasp postures appropriately
		g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
		g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
		
		# Set the approach and retreat parameters as desired
		g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
		g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
		
		# Set the first grasp pose to the input pose
		g.grasp_pose = initial_pose_stamped
		
		# Pitch angles to try
		pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
		
		# Yaw angles to try
		yaw_vals = [0]
		
		# A list to hold the grasps
		grasps = []
		
		# Generate a grasp for each pitch and yaw angle
		for y in yaw_vals:
			for p in pitch_vals:
				# Create a quaternion from the Euler angles
				q = quaternion_from_euler(0, p, y)
				
				# Set the grasp pose orientation accordingly
				g.grasp_pose.pose.orientation.x = q[0]
				g.grasp_pose.pose.orientation.y = q[1]
				g.grasp_pose.pose.orientation.z = q[2]
				g.grasp_pose.pose.orientation.w = q[3]
				
				# Set and id for this grasp (simply needs to be unique)
				g.id = str(len(grasps))
				
				# Set the allowed touch objects to the input list
				g.allowed_touch_objects = allowed_touch_objects
				
				# Don't restrict contact force
				g.max_contact_force = 0
				
				# Degrade grasp quality for increasing pitch angles
				g.grasp_quality = 1.0 - abs(p)
				
				# Append the grasp to the list
				grasps.append(deepcopy(g))
		
		# Return the list
		return grasps
	
	# Generate a list of possible place poses
	def make_places(self, init_pose):
		# Initialize the place location as a PoseStamped message
		place = PoseStamped()
		
		# Start with the input place pose
		place = init_pose
		
		# A list of x shifts (meters) to try
		x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
		
		# A list of y shifts (meters) to try
		y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
		
		pitch_vals = [0]
		
		# A list of yaw angles to try
		yaw_vals = [0]
		
		# A list to hold the places
		places = []
		
		# Generate a place pose for each angle and translation
		for y in yaw_vals:
			for p in pitch_vals:
				for y in y_vals:
					for x in x_vals:
						place.pose.position.x = init_pose.pose.position.x + x
						place.pose.position.y = init_pose.pose.position.y + y
						
						# Create a quaternion from the Euler angles
						q = quaternion_from_euler(0, p, y)
						
						# Set the place pose orientation accordingly
						place.pose.orientation.x = q[0]
						place.pose.orientation.y = q[1]
						place.pose.orientation.z = q[2]
						place.pose.orientation.w = q[3]
						
						# Append this place pose to the list
						places.append(deepcopy(place))
		
		# Return the list
		return places
	
	# Set the color of an object
	def setColor(self, name, r, g, b, a = 0.9):
		# Initialize a MoveIt color object
		color = ObjectColor()
		
		# Set the id to the name given as an argument
		color.id = name
		
		# Set the rgb and alpha values given as input
		color.color.r = r
		color.color.g = g
		color.color.b = b
		color.color.a = a
		
		# Update the global color dictionary
		self.colors[name] = color
	
	# Actually send the colors to MoveIt!
	def sendColors(self):
		# Initialize a planning scene object
		p = PlanningScene()
		
		# Need to publish a planning scene diff
		p.is_diff = True
		
		# Append the colors from the global color dictionary
		for color in self.colors.values():
			p.object_colors.append(color)
		
		# Publish the scene diff
		self.scene_pub.publish(p)


class GazeboClient(object):
	def __init__(self, skip_models = None):
		self.skip_models = skip_models if skip_models is not None else []
	
	def get_model_states(self):
		"""

		:rtype: object
		"""
		model_state = False
		while not model_state:
			msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout = 10.0)
			try:
				if msg:
					return msg
			except:
				pass
	
	def load_gazebo_sdf_models(self, model = None, pkg_name = None, reference_frame = 'world'):
		if not model: return
		pkg_name = rospkg.RosPack().get_path(
			pkg_name) + "/models/" if pkg_name is not None else '/home/mtb/.gazebo/models'
		
		rospy.wait_for_service('/gazebo/spawn_sdf_model')
		for i in range(len(model['name'])):
			with open(pkg_name + model['path'][i] + "/" + "model.sdf", "r") as xml_file:
				model_xml = xml_file.read().replace('\n', '')
			try:
				spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
				spawn_sdf(model['name'][i], model_xml, "/", model['pose'][i], reference_frame)
			
			
			except rospy.ServiceException, e:
				rospy.logerr("Spawn SDF service call failed: {0}".format(e))
		return self.get_model_states().name
	
	def initial_load_gazebo_models(self):
		model = OrderedDict()
		model_path = 'rbt_baxter_simulation'
		model['path'] = ['cube_green', 'cube_green', 'cube_red', 'cube_red',
		                 'cube_blue', 'cube_blue']
		model['name'] = ['cube_green_0', 'cube_green_1', 'cube_red_0',
		                 'cube_red_1', 'cube_blue_0', 'cube_blue_1']
		# model['pose'] = [Pose(position = Point(x = 0.75, y = 0.0, z = 0.0))]
		model['pose'] = []
				
		initial_x_pose = 0.75
		initial_y_pose = 0.25
		gap = 0.15
		for i in range(len(model['name'])):
			model['pose'].append(Pose(position = Point(x = initial_x_pose, y = initial_y_pose, z = 0.8)))
			initial_y_pose -= gap
			if initial_y_pose <= -0.45:
				initial_x_pose += gap
				initial_y_pose = 0.25
		
		return self.load_gazebo_sdf_models(model, model_path)
	
	def delete_gazebo_sdf_models(self, model_name = None):
		
		if model_name is None:
			model_name = self.get_model_states().name
			model_name = [names for names in model_name if names not in self.skip_models]
			if not model_name: return
		
		for models in model_name:
			try:
				delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
				delete_model(models)
			except rospy.ServiceException, e:
				rospy.loginfo("Delete Model service call failed: {0}".format(e))
	
	def shuffle_models(self, model_name = None, model_path = None):
		if not model_name: return
		x_min, x_max = 0.6, 0.85
		y_min, y_max = -0.2, 0.4
		
		x_list, y_list = [x_min, x_max], [y_min, y_max]
		positions = []
		while True:
			
			points = [round(random.uniform(x_list[0], x_list[1]), 3), round(random.uniform(y_list[0], y_list[1]), 3),
			          0.5]
			collision = False
			for pose in positions:
				if sqrt((pose[0] - points[0]) ** 2 + (pose[1] - points[1]) ** 2) <= 0.15:
					collision = True
					break
			if not collision:
				positions.append(points)
			if len(positions) == len(model_name): break
		
		for i in range(len(positions)):
			positions[i] = Pose(position = Point(*positions[i]))
		
		model = OrderedDict()
		model['name'] = model_name
		model['pose'] = positions
		model['path'] = model_path
		self.load_gazebo_sdf_models(model, 'bot_gazebo', 'world')


if __name__ == "__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	
	rospy.init_node('pick_and_place')
	
	rs = rbt_baxter_interface.RobotEnable(CHECK_VERSION)
	print("Enabling robot... ")
	
	pick_place = PickAndPlace(limb = 'left')
	# pick_place.gazebo_client.initial_load_gazebo_models()
	pick_place.get_scene()
	
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	


# class ArmClient(object):
# 	def __init__(self, limb = 'left'):
# 		ns = 'robot/limb/' + limb + '/'
# 		self._client = actionlib.SimpleActionClient(
# 			ns + "follow_joint_trajectory",
# 			FollowJointTrajectoryAction,
# 		)
# 		self._goal = FollowJointTrajectoryGoal()
# 		self._goal_time_tolerance = rospy.Time(0.1)
# 		self._goal.goal_time_tolerance = self._goal_time_tolerance
# 		server_up = self._client.wait_for_server(timeout = rospy.Duration(10.0))
# 		if not server_up:
# 			rospy.logerr("Timed out waiting for Joint Trajectory"
# 			             " Action Server to connect. Start the action server"
# 			             " before running example.")
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
# 	def wait(self, timeout = 15.0):
# 		self._client.wait_for_result(timeout = rospy.Duration(timeout))
#
# 	def result(self):
# 		return self._client.get_result()
#
# 	def clear(self, limb):
# 		self._goal = FollowJointTrajectoryGoal()
# 		self._goal.goal_time_tolerance = self._goal_time_tolerance
# 		self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
# 		                                     ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]
