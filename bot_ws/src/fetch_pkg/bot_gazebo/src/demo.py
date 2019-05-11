#!/usr/bin/env python

import copy
import sys
import actionlib
import rospy
import rospkg

from math import sin, cos, sqrt
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandGoal, GripperCommandAction
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.listener import TransformListener
import tf
from std_msgs.msg import String, Header

import numpy as np

import cv2
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from collections import OrderedDict
from bot_gazebo.srv import ImageState
import rosbag
from gazebo_msgs.srv import (
	SpawnModel,
	DeleteModel,
)
import random

target_frame = '/map'
source_frame = '/base_link'

cube_idx = {0: 'red', 1: 'green', 2: 'blue'}
cube_type = {'blue': ['cube_blue_0', 'cube_blue_1'],
             'red': ['cube_red_0', 'cube_red_1'],
             'green': ['cube_green_0', 'cube_green_1']}
cube_map_pose = OrderedDict([('cube_blue_1', [1.4, -0.3, 0.83]), ('cube_blue_2', [1.5, -0.15, 0.83]),
                             ('cube_red_1', [1.4, 0.2, 0.83]), ('cube_red_2', [1.6, 0.2, 0.83]),
                             ('cube_green_1', [1.5, 0.0, 0.83]), ('cube_green_2', [1.5, 0.32, 0.83])])


class Demo(object):
	def __init__(self):
		# rospy.init_node('demo')

		# self.move_base = MoveBaseClient()
		# self.torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])

		self.bridge = CvBridge()
		self.gazebo_client = GazeboClient(skip_models = None)
		self.head_action = PointHeadClient()
		self.grasping_client = GraspingClient()
		self.skipped = []
		self.m = 0
		rospy.wait_for_service('state', timeout = 10.0)

	def cubes_bottom_top(self):
		self.is_top = []
		self.is_bottom = []
		self.combination = []
		self.all_sequence = []
		self.skipped = []
		# self.move_base.goto(.8, 0.0, 0.0)
		# self.torso_action.move_to([0.4, ])
		self.head_action.look_at(0.6, 0.1, 0.4, "map")
		self.grasping_client.tuck()
		rospy.sleep(1.0)

	def get_pose_init(self):
		model = self.get_model_states()
		self.cube_map_pose = OrderedDict()

		for i in range(len(model.name) - 6, len(model.name)):
			self.cube_map_pose[model.name[i]] = [model.pose[i].position.x, model.pose[i].position.y,
			                                     model.pose[i].position.z]

	def reset(self):
		state_data = 'error'
		while state_data == 'error':
			client = rospy.ServiceProxy('state', ImageState)
			state = client()

			if state.saved == 'success':
				with rosbag.Bag('state.bag', 'r') as inputbag:
					for topic, msg, timestamp in inputbag.read_messages(topics = '/head_camera/rgb/image_color'):
						image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
						image = cv2.resize(image, (84, 84))
						return image

	def get_model_states(self):
		model_state = False
		while not model_state:
			msg = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout = 10.0)
			try:
				if msg:
					return msg
			except:
				pass

	def step(self, seq_list):
		self.get_pose_init()
		update = False
		start = rospy.Time.now()
		try:
			while rospy.Time.now() - start <= rospy.Duration(3600.0):
				min_cube_detect = 7 - len(self.skipped) if self.skipped else 6
				max_cube_detect = 6
				detect_trial = 0
				detected = False
				while detect_trial <= 5.0:
					self.grasping_client.updateScene()


					if min_cube_detect <= len(self.grasping_client.cubes) <= max_cube_detect:
						detected = True
						break
					detect_trial += 1

				if not detected:
					return None, None, True, False, 0, 0, 0, 0
				self.positions = OrderedDict()

				for i in range(len(self.grasping_client.cubes)):
					pose = self.grasping_client.cubes[i]
					error_list = []
					# Base to map matrix transformation
					ref_position = self.grasping_client.get_transform_pose(pose)

					for keys in self.cube_map_pose:
						if keys not in self.positions:
							error_list.append([(ref_position.x - self.cube_map_pose[keys][0]) ** 2,
							                   (ref_position.y - self.cube_map_pose[keys][1]) ** 2,
							                  (ref_position.z - self.cube_map_pose[keys][2]) ** 2])

					error_list = np.array(error_list)

					index = int(np.argmin(np.sum(error_list, axis = 1)))

					self.positions[self.cube_map_pose.keys()[index]] = pose
					delete_keys = self.cube_map_pose.keys()[index]
					del self.cube_map_pose[delete_keys]

				move_cube_start = cube_type[cube_idx[seq_list[0]]]
				move_cube_dest = cube_type[cube_idx[seq_list[1]]]
				selected = {'top': [], 'bottom': []}

				if (move_cube_start[0], move_cube_dest[1]) in self.all_sequence:
					next_state = self.reset()
					reward = 0.0
					terminal = False
					update = True
					# print('Already the sequence has beeen completed')
					return next_state, reward, terminal, update, 0, 0, 0, 0

				if move_cube_start[0] not in self.is_bottom:
					selected['top'] = [move_cube_start[0], self.positions[move_cube_start[0]]]
				else:
					# print('The start cube is on bottom')
					next_state = self.reset()
					reward = 0.0
					terminal = False
					update = True
					# print('Already the sequence has beeen completed')
					return next_state, reward, terminal, update, 0, 0, 0, 0

				if move_cube_dest[1] not in self.is_top and move_cube_dest[1] not in self.is_bottom:
					selected['bottom'] = [move_cube_dest[1], self.positions[move_cube_dest[1]]]
				else:
					next_state = self.reset()
					reward = 0.0
					terminal = False
					update = True
					# print('Already the sequence has beeen completed')
					return next_state, reward, terminal, update, 0, 0, 0, 0

				sequence_object_list = [self.positions.keys().index(selected['top'][0]),
				                        self.positions.keys().index(selected['bottom'][0])]

				pick_seq = self.grasping_client.get_sequence(sequence_object_list)
				cube, grasps = pick_seq['start']
				if cube is None:
					print('No cube')
					return None, None, True, False, 0, 0, 0, 0

				picked = False

				grasp_attempt = 1
				if self.grasping_client.pick(cube, grasps):
					picked = True
				if picked:
					grasp_success = 1

					placed = False

					cube_target, grasps_target = pick_seq['target']
					pose = PoseStamped()
					pose.pose = cube_target.primitive_poses[0]
					pose.pose.position.z += 0.09
					pose.header.frame_id = cube_target.header.frame_id
					if self.grasping_client.place(cube, pose):
						placed = True
					if placed:
						self.grasping_client.tuck()
						rospy.sleep(1.0)
						if move_cube_start[0] not in self.skipped:
							self.skipped.append(move_cube_start[0])
						if move_cube_dest[1] not in self.skipped:
							self.skipped.append(move_cube_dest[1])

						model = self.get_model_states()
						position_start = model.name.index(move_cube_start[0])
						position_target = model.name.index(move_cube_dest[1])
						gazebo_state_start = model.pose[position_start]
						gazebo_state_target = model.pose[position_target]

						if gazebo_state_start.position.z >= 0.45 and \
								gazebo_state_target.position.x - 0.02 <= gazebo_state_start.position.x <= gazebo_state_target.position.x + 0.04 and \
								gazebo_state_target.position.y - 0.02 <= gazebo_state_start.position.y <= gazebo_state_target.position.y + 0.04:

							self.is_bottom.append(move_cube_dest[1])
							if move_cube_start[0] not in self.is_top:
								self.is_top.append(move_cube_start[0])
							remove_index = None
							for i in range(len(self.all_sequence)):
								if move_cube_start[0] in self.all_sequence[i]:
									remove_index = i
							if remove_index: self.all_sequence.pop(remove_index)

							if (move_cube_start[0], move_cube_dest[1]) not in self.all_sequence:
								self.all_sequence.append((move_cube_start[0], move_cube_dest[1]))

							reward = 1.0 if move_cube_start[0][:-2] == move_cube_dest[1][:-2] else -1.0

							new_state = self.reset()
							terminal = len(self.all_sequence) == 3
							update = True
							# 			# print('The cube has been placed correcly')
							self.skipped.append(move_cube_start[0])
							self.grasping_client.gripper_opening()
							self.grasping_client.updateScene(remove_collision = True)
							return new_state, reward, terminal, update, 1, 1, 1, 1
						#
						else:
							next_state = None
							reward = None
							terminal = True
							update = False
							self.grasping_client.gripper_opening()
							# self.grasping_client.updateScene(remove_collision = True)
							return next_state, reward, terminal, update, 1, 1, 1, 0
					else:

						self.grasping_client.gripper_opening()
						self.grasping_client.updateScene(remove_collision = True)

						next_state = None
						reward = None
						terminal = True
						update = False
						return next_state, reward, terminal, update, 1, 1, 1, 0
				#
				else:
					self.grasping_client.gripper_opening()
					# self.grasping_client.updateScene(remove_collision = True)
					next_state = None
					reward = 0.0
					terminal = True
					update = False

					return next_state, reward, terminal, update, 1, 0, 0, 0

		except:
			return None, None, True, False, 0, 0, 0, 0


class MoveBaseClient(object):

	def __init__(self):
		self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
		# rospy.loginfo("Waiting for move_base...")
		self.client.wait_for_server()

	def goto(self, x, y, theta, frame = "map"):
		move_goal = MoveBaseGoal()
		move_goal.target_pose.pose.position.x = x
		move_goal.target_pose.pose.position.y = y
		move_goal.target_pose.pose.orientation.z = sin(theta / 2.0)
		move_goal.target_pose.pose.orientation.w = cos(theta / 2.0)
		move_goal.target_pose.header.frame_id = frame
		move_goal.target_pose.header.stamp = rospy.Time.now()

		# TODO wait for things to work
		self.client.send_goal(move_goal)
		self.client.wait_for_result()


# Send a trajectory to controller
class FollowTrajectoryClient(object):

	def __init__(self, name, joint_names):
		self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
		                                           FollowJointTrajectoryAction)
		# rospy.loginfo("Waiting for %s..." % name)
		self.client.wait_for_server()
		self.joint_names = joint_names

	def move_to(self, positions, duration = 5.0):
		if len(self.joint_names) != len(positions):
			print("Invalid trajectory position")
			return False
		trajectory = JointTrajectory()
		trajectory.joint_names = self.joint_names
		trajectory.points.append(JointTrajectoryPoint())
		trajectory.points[0].positions = positions
		trajectory.points[0].velocities = [0.0 for _ in positions]
		trajectory.points[0].accelerations = [0.0 for _ in positions]
		trajectory.points[0].time_from_start = rospy.Duration(duration)
		follow_goal = FollowJointTrajectoryGoal()
		follow_goal.trajectory = trajectory

		self.client.send_goal(follow_goal)
		self.client.wait_for_result()


# Point the head using controller
class PointHeadClient(object):

	def __init__(self):
		self.client = actionlib.SimpleActionClient("head_controller/point_head", PointHeadAction)
		# rospy.loginfo("Waiting for head_controller...")
		self.client.wait_for_server()

	def look_at(self, x, y, z, frame, duration = 1.0):
		goal = PointHeadGoal()
		goal.target.header.stamp = rospy.Time.now()
		goal.target.header.frame_id = frame
		goal.target.point.x = x
		goal.target.point.y = y
		goal.target.point.z = z
		goal.min_duration = rospy.Duration(duration)
		self.client.send_goal(goal)
		self.client.wait_for_result()


class GraspingClient(object):

	def __init__(self):
		self.scene = PlanningSceneInterface("base_link")
		self.pickplace = PickPlaceInterface("arm", "gripper", verbose = True)
		self.move_group = MoveGroupInterface("arm", "base_link")

		find_topic = "basic_grasping_perception/find_objects"
		# rospy.loginfo("Waiting for %s..." % find_topic)
		self.find_client = actionlib.SimpleActionClient(find_topic, FindGraspableObjectsAction)
		self.find_client.wait_for_server()
		self.cubes = []
		self.tf_listener = TransformListener()
		self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
		self.gripper_client.wait_for_server()
		rospy.loginfo("...connected.")
		rospy.sleep(2.0)

	def updateScene(self, remove_collision = False):
		if remove_collision:
			for name in self.scene.getKnownCollisionObjects():
				self.scene.removeCollisionObject(name, False)
			for name in self.scene.getKnownAttachedObjects():
				self.scene.removeAttachedObject(name, False)
			self.scene.waitForSync()
			return

		# find objects
		self.cubes = []
		goal = FindGraspableObjectsGoal()
		goal.plan_grasps = True
		self.find_client.send_goal(goal)
		self.find_client.wait_for_result(rospy.Duration(5.0))
		find_result = self.find_client.get_result()

		# remove previous objects
		for name in self.scene.getKnownCollisionObjects():
			self.scene.removeCollisionObject(name, False)
		for name in self.scene.getKnownAttachedObjects():
			self.scene.removeAttachedObject(name, False)
		self.scene.waitForSync()

		# insert objects to scene
		idx = -1
		for obj in find_result.objects:
			print(idx)
			idx += 1
			obj.object.name = "object%d" % idx
			self.scene.addSolidPrimitive(obj.object.name,
			                             obj.object.primitives[0],
			                             obj.object.primitive_poses[0],
			                             wait = False)
			self.cubes.append(obj.object.primitive_poses[0])

		for obj in find_result.support_surfaces:
			# extend surface to floor, and make wider since we have narrow field of view
			height = obj.primitive_poses[0].position.z
			obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
			                                1.5,  # wider
			                                obj.primitives[0].dimensions[2] + height]
			obj.primitive_poses[0].position.z += -height / 2.0

			# add to scene
			self.scene.addSolidPrimitive(obj.name,
			                             obj.primitives[0],
			                             obj.primitive_poses[0],
			                             wait = False)

		self.scene.waitForSync()

		# store for grasping
		self.objects = find_result.objects
		self.surfaces = find_result.support_surfaces

	def getGraspableCube(self):
		graspable = None
		for obj in self.objects:
			# need grasps
			if len(obj.grasps) < 1:
				continue
			# check size
			if obj.object.primitives[0].dimensions[0] < 0.05 or \
					obj.object.primitives[0].dimensions[0] > 0.07 or \
					obj.object.primitives[0].dimensions[0] < 0.05 or \
					obj.object.primitives[0].dimensions[0] > 0.07 or \
					obj.object.primitives[0].dimensions[0] < 0.05 or \
					obj.object.primitives[0].dimensions[0] > 0.07:
				continue
			# has to be on table
			if obj.object.primitive_poses[0].position.z < 0.5:
				continue
			return obj.object, obj.grasps
		# nothing detected
		return None, None

	def get_sequence(self, seq_list):
		start = [self.objects[seq_list[0]].object, self.objects[seq_list[0]].grasps] or [None, None]

		target = [self.objects[seq_list[1]].object, self.objects[seq_list[1]].grasps] or [None, None]

		res = {'start': start, 'target': target}

		return res

	def get_transform_pose(self, pose):
		point = PoseStamped()
		point.header.frame_id = 'base_link'
		point.header.stamp = rospy.Time()
		point.pose = pose

		return self.tf_listener.transformPose('/map', point).pose.position

	def getPlaceLocation(self):
		pass

	def pick(self, block, grasps):
		success, pick_result = self.pickplace.pick_with_retry(block.name,
		                                                      grasps,
		                                                      retries = 5,
		                                                      support_name = block.support_surface,
		                                                      scene = self.scene)
		self.pick_result = pick_result
		return success

	def place(self, block, pose_stamped):
		places = list()
		l = PlaceLocation()
		l.place_pose.pose = pose_stamped.pose
		l.place_pose.header.frame_id = pose_stamped.header.frame_id

		# copy the posture, approach and retreat from the grasp used
		l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
		l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
		l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
		places.append(copy.deepcopy(l))
		# create another several places, rotate each by 360/m degrees in yaw direction
		m = 16  # number of possible place poses
		pi = 3.141592653589
		for i in range(0, m - 1):
			l.place_pose.pose = rotate_pose_msg_by_euler_angles(l.place_pose.pose, 0, 0, 2 * pi / m)
			places.append(copy.deepcopy(l))

		success, place_result = self.pickplace.place_with_retry(block.name,
		                                                        places,
		                                                        scene = self.scene,
		                                                        retries = 5)
		# print(success)
		return success

	def tuck(self):
		joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
		          "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
		pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
		# pose =[ 1.58, -0.056, -0.01, -1.31, -0.178, -0.13, 0.16]
		gripper_goal = GripperCommandGoal()
		gripper_goal.command.max_effort = 0.0
		gripper_goal.command.position = 0.09
		self.gripper_client.send_goal(gripper_goal)

		self.gripper_client.wait_for_result(rospy.Duration(5))
		start = rospy.Time.now()
		while rospy.Time.now() - start <= rospy.Duration(10.0):  # rospy.is_shutdown():
			result = self.move_group.moveToJointPosition(joints, pose, 0.02)
			if result.error_code.val == MoveItErrorCodes.SUCCESS:
				return
		return

	def gripper_opening(self, opening = 0.09):
		gripper_goal = GripperCommandGoal()
		gripper_goal.command.max_effort = 0.0
		gripper_goal.command.position = opening
		self.gripper_client.send_goal(gripper_goal)
		self.gripper_client.wait_for_result(rospy.Duration(5))


class GazeboClient(object):
	def __init__(self, skip_models = None):
		self.skip_models = skip_models if skip_models is not None else []

	def get_model_states(self):
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
		model_path = 'bot_gazebo'
		model['path'] = ['building', 'cafe_table_scaled', 'cube_green', 'cube_green', 'cube_red', 'cube_red',
		                 'cube_blue', 'cube_blue']
		model['name'] = ['building_0', 'cafe_table_scaled_0', 'cube_green_0', 'cube_green_1', 'cube_red_0',
		                 'cube_red_1', 'cube_blue_0', 'cube_blue_1']
		model['pose'] = [Pose(position = Point(x = 0.0, y = 0.0, z = 0.0)),
		                 Pose(position = Point(x = 0.8, y = 0.0, z = 0.0))]

		# model['name'] = ['cube_green','cube_green', 'cube_red', 'cube_red','cube_blue', 'cube_blue']
		# model['pose'] = []

		initial_x_pose = 0.85
		initial_y_pose = 0.35
		gap = 0.15
		for i in range(len(model['name'])):
			model['pose'].append(Pose(position = Point(x = initial_x_pose, y = initial_y_pose, z = 0.6)))
			initial_y_pose -= gap
			if initial_y_pose <= -0.25:
				initial_x_pose += gap
				initial_y_pose = 0.4

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

	def shuffle_models(self, model_name = None, model_path=None):
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
#
# if __name__ == '__main__':
# 	rospy.init_node('demo')

	head = PointHeadClient()
	head.look_at(0.6, -0.05, 0.4, "map")

	# gazebo = GazeboClient()
	# gazebo.initial_load_gazebo_models()
	
	# torso = FollowTrajectoryClient(name = 'torso_controller', joint_names = ["torso_lift_joint"])
	# torso.move_to([0.8],5.0)
	#
	# gripper = GraspingClient()
	# gripper.updateScene()
	#
	
	
	
	
	
# 	a = Demo()
# 	a.head_action.look_at(0.6, -0.05, 0.4, "map")


# waypoints = [0.669, -0.45, 2.23, 1.21, 2.8, 0.76, -2.3]
# joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
#           "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
# move_group = MoveGroupInterface("arm", "map")
# move_group.moveToJointPosition(joints, waypoints, 0.03)
