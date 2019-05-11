# coding=utf-8

import os
import pybullet as p
import pybullet_data
import numpy as np
from collections import namedtuple
from itertools import combinations
import time
import os

from utils import Pd, mkdir

ROBOT = 'JACO'
# ROBOT ='UR5'

if ROBOT == 'UR5':
    MOVABLE_ARM_JOINTS = [1, 2, 3, 4, 5, 6]
    DISABLE_COLLSIONS = [(6, 9), (10, 13), (10, 15), (10, 17), (12, 16), (14, 18)]
    MOVABLE_GRIPPER_JOINTS = [11, 13, 15, 16, 17, 18]
elif ROBOT == 'JACO':
    MOVABLE_ARM_JOINTS = [3, 5, 7, 9, 11, 13]
    DISABLE_COLLSIONS = [(3, 5), (5, 7), (7, 9), (9, 11), (11, 13), (13, 15), (13, 16), (13, 18), (13, 19), (13, 21),
                         (13, 22)]
    MOVABLE_GRIPPER_JOINTS = [16, 19, 22]

MAX_DISTANCE = 0.0
EPSILON = 0.05
SLEEP = 0.001

DOF = 9

PHYSICS = 0.005
STEP = 5

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])
JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
                                       'jointReactionForces', 'appliedJointMotorTorque'])

LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])

BodyInfo = namedtuple('BodyInfo', ['base_link', 'robot_name'])
TreeNode = namedtuple('TreeNode', ['config', 'parent'])

World = namedtuple('World', ['body', 'pose', 'joints', 'configuration'])

# Camera parameters
camTargetPos = [0, 0, 0]
cameraUp = [0, 0, 1]
cameraPos = [1, 1, 1]

pitch = -50
roll = 10
yaw = 70
upAxisIndex = 2
camDistance = 1.5
pixelWidth = 256
pixelHeight = 256
nearPlane = 0.01
farPlane = 100
fov = 60

CONNEXION_MODE = {'gui': p.GUI,
                  'direct': p.DIRECT}


class Interface(object):

    def __init__(self, mode = 'gui', save = False, filename = None):
        self.client_id = -1
        self.movable_a_joints = MOVABLE_ARM_JOINTS
        self.movable_g_joints = MOVABLE_GRIPPER_JOINTS
        self.filename = filename
        self.mode = mode
        if save:
            velocity_filename = filename
            joints = ['joints_%d' % i for i in self.movable_a_joints + self.movable_g_joints]
            self.vel = Pd(velocity_filename, columns = joints, shape = (0, len(joints)))

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()

    def connect(self):
        """Connect to pybullet"""
        p.connect(p.SHARED_MEMORY)

        self.client_id = p.connect(CONNEXION_MODE[self.mode])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.enable_gravity()
        p.setTimeStep(PHYSICS)

        # p.setPhysicsEngineParameter(numSolverIterations = 5)
        # p.setPhysicsEngineParameter(fixedTimeStep = 0.005)
        # p.setPhysicsEngineParameter(numSubSteps = 10)
        # p.setPhysicsEngineParameter(fixedTimeStep = 1.0 / 60., solverResidualThreshold = 1 - 10,
        #                             numSolverIterations = 50, numSubSteps = 4)

        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        # p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        # p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId = self.client_id)
        # p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, False, physicsClientId = self.client_id)

    def disconnect(self):
        """Disconnect the pybullet"""
        p.disconnect(physicsClientId = self.client_id)

    def load_model(self, path, pose = None, fixed_base = True, scaling = 1.0):
        """
        Load the objects in the scene
        :param path: str
            Path to the object
        :param pose: Pose
            Position and Quaternion
        :param fixed_base: str
            Fixed in the scene
        :param scaling: float
            Scale object
        :return: int
            ID of the loaded object
        """
        if path.endswith('.urdf'):
            body = p.loadURDF(path, useFixedBase = fixed_base, flags = 0, globalScaling = scaling,
                              physicsClientId = self.client_id)
        elif path.endswith('.sdf'):
            body = p.loadSDF(path, physicsClientId = self.client_id)
        elif path.endswith('.xml'):
            body = p.loadMJCF(path, physicsClientId = self.client_id)
        elif path.endswith('.bullet'):
            body = p.loadBullet(path, physicsClientId = self.client_id)
        else:
            raise ValueError(path)
        if pose is not None:
            self.set_pose(body, pose)
        return body

    def set_pose(self, body, pose):
        """
        Set the positions of the objects int he scene
        :param body:int
            body's ID
        :param pose: Pose
            Position and Quaternion
        :return: None
        """
        (point, quat) = pose
        p.resetBasePositionAndOrientation(body, point, quat, physicsClientId = self.client_id)

    def get_position(self, body):
        """
        Get the position of the ID
        :param body:int
            body's ID
        :return: tuple
            Tuple of Position and Quaternion
        """
        return p.getBasePositionAndOrientation(body, self.client_id)

    def get_num_joints(self, body):
        """
        Get the position of the ID
        :param body:int
            body's ID
        :return: int
            The number of joints in the body
        """
        return p.getNumJoints(body, physicsClientId = self.client_id)

    def get_joint_info(self, body, joint):
        """
        Get the joint info
        :param body:int
            body's ID
        :param joint: int
            robot's joint ID
        :return: JointInfo
            Joint name,damping,limits...
        """
        return JointInfo(*p.getJointInfo(body, joint, physicsClientId = self.client_id))

    def get_joint_state(self, body, joints):
        """
        Get the joint state
        :param body:int
            body's ID
        :param joints: list or int
            robot's joint ID
        :return: JointState
            Tuple of JointState(joint positions,velocities,applied torques...)
        """
        if isinstance(joints, list):
            return [JointState(*p.getJointState(body, joint, physicsClientId = self.client_id)) for joint in joints]
        elif isinstance(joints, int):
            return JointState(*p.getJointState(body, joints, physicsClientId = self.client_id))

    @staticmethod
    def get_body_info(body):
        """
        Get the body info
        :param body:int
            body's ID
        :return: class
            Return base name of the body
        """
        return BodyInfo(*p.getBodyInfo(body))

    def get_num_bodies(self):
        """
        Get the number of bodies in the physics server
        :return: int
            The number of bodies present
        """
        return p.getNumBodies(physicsClientId = self.client_id)

    def get_link_from_name(self, body, name):
        """
        Get the link ID from name
        :param body: int
            body's ID
        :param name: str
            link's name
        :return: int
            ID of the link
        """
        for joint in range(self.get_num_joints(body)):
            if name == self.get_joint_info(body, joint).linkName:
                return joint
        return -1

    def get_movable_joints(self, body):
        """
        Get the movable joints of the body
        :param body: int
            body's ID
        :return: list
            Joints that can be moved
        """
        return [joint for joint in range(self.get_num_joints(body)) if
                self.get_joint_info(body, joint).jointType != p.JOINT_FIXED]

    def set_joint_position(self, body, joint, value):
        """
        Reset joint position to the value position
        :param body: int
            body's ID
        :param joint: int
            robot's joint ID
        :param value: list
            Joint angle value
        """
        p.resetJointState(body, joint, value, physicsClientId = self.client_id)

    def set_joint_positions(self, body, joints, values):
        assert len(joints) == len(values)
        for joint, value in zip(joints, values):
            self.set_joint_position(body, joint, value)

    def get_euler_from_quaternion(self, quaternion):
        """
        Get the quaternion parameters from euler angles
        :param quaternion: Quaternion
        :return: Euler angles
        """
        return p.getEulerFromQuaternion(quaternion, physicsClientId = self.client_id)

    def get_link_state(self, body, link):
        """
        Get the state of the body's link
        :param body: int
            body's ID
        :param link: int
            body's link
        :return: LinkState
            link world position, orientation...
        """
        return LinkState(*p.getLinkState(body, link, physicsClientId = self.client_id))

    def get_link_pose(self, body, link):
        """
        Get the pose of a link
        :param body: int
            body's ID
        :param link: int
            body's link
        :return: tuple
            Link Pose
        """
        if link == -1:
            return None
        link_state = self.get_link_state(body, link)
        return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

    def calculate_inverse_kinematics(self, body, eef_link, target_pose, obstacles = None, max_iterations = 200,
                                     tolerance = 1e-3):
        """
        Calculate the inverse kinematics for an arm
        :param body:int
            body's ID
        :param eef_link: int
            link's name
        :param target_pose: list
            List of target position and quaternion
        :param max_iterations: int
            Maximum iterations for IK calcul
        :param tolerance: float
            Tolerance between the arm positions and the target positions
        :return: list
            arm positions
        """
        (target_point, target_quat) = target_pose
        current_config = [state.jointPosition for state in self.get_joint_state(body, range(self.get_num_joints(body)))]

        for iterations in range(max_iterations):
            if target_quat is None:
                kinematic_conf = p.calculateInverseKinematics(body, eef_link, target_point,
                                                              physicsClientId = self.client_id)
            else:
                kinematic_conf = p.calculateInverseKinematics(body, eef_link, target_point, target_quat,
                                                              physicsClientId = self.client_id)
            if kinematic_conf:
                kinematic_conf = kinematic_conf[:len(
                    self.movable_a_joints)]  # [float(q) for q in kinematic_conf[:len(self.movable_a_joints)]]
            else:
                continue

            self.set_joint_positions(body, self.movable_a_joints, kinematic_conf)
            link_point, link_quat = self.get_link_pose(body, eef_link)
            if np.allclose(link_point, target_point, atol = tolerance, rtol = 0):
                if self.violates_limits(body, kinematic_conf):
                    continue
                if self.pairs_link_in_collision(body):
                    continue
                if self.bodies_in_collision(body, obstacles):
                    continue
                break
        else:

            return None

        self.set_joint_positions(body, range(self.get_num_joints(body)), current_config)
        return kinematic_conf[:6] if kinematic_conf else None

    def violates_limits(self, body, kinematic_conf):
        """
        Verify if the limits of the joints
        :param body: int
            body's ID
        :param kinematic_conf: list
            Arm angle
        :return: bool
            True if the angles are violated
        """
        return not all([up_limit >= q_inv >= low_limit for (q_inv, (low_limit, up_limit)) in
                        zip(kinematic_conf, self.get_joint_limits(body, self.movable_a_joints))])

    def get_joint_limits(self, body, joint):
        """
        Get the joints angles limits
        :param body: int
            body's ID
        :param joint: int or list
            robot's joint ID
        :return: list
            joints limits from JointState
        """
        limits = []
        if isinstance(joint, list):
            for joint_ in joint:
                joint_info = self.get_joint_info(body, joint_)
                limits.append((joint_info.jointLowerLimit, joint_info.jointUpperLimit))
            return limits
        elif isinstance(joint, int):
            joint_info = self.get_joint_info(body, joint)
            limits.append((joint_info.jointLowerLimit, joint_info.jointUpperLimit))

        return limits

    def pairs_link_in_collision(self, body):
        """
        Get the pair of links in collision
        :param body: int
            body's ID
        :return: bool
        """

        links = self.retrieve_all_links(body)
        pair_links = combinations(range(len(links)), 2)

        for (link1, link2) in pair_links:
            if not self.adjacent(link1, link2):
                if (link1, link2) in DISABLE_COLLSIONS:
                    continue
                elif self.check_link_collision(body, link1, body, link2):
                    return True
        return False

    def retrieve_all_links(self, body):
        """
        Get all the links from body
        :param body: int
            body's ID
        :return: list
            List of all links
        """
        links = []
        for j in range(self.get_num_joints(body)):
            links.append((j, self.get_joint_info(body, j).linkName))
        return links

    def check_link_collision(self, body1, link1, body2, link2, max_distance = MAX_DISTANCE):
        """
        Check collision between two links
        :param body1: int
            body's ID
        :param link1: int
            body's link
        :param body2: int
            body's ID
        :param link2: int
            body's link
        :param max_distance: float
            distance of contact between the links
        :return: bool
            Distance
        """
        return len(p.getClosestPoints(bodyA = body1, bodyB = body2, distance = max_distance,
                                      linkIndexA = link1, linkIndexB = link2,
                                      physicsClientId = self.client_id)) != 0

    def bodies_in_collision(self, body, obstacles = None):
        if obstacles is None:
            return False
            # obstacles = filter(lambda b: b is not body, range(self.get_num_bodies()))

        return any(len(p.getClosestPoints(body, other_body, MAX_DISTANCE)) for other_body in obstacles)

    @staticmethod
    def adjacent(*args):
        """
        Check if the links are adjacents
        :param args: link ID
        :return: bool
        """
        return True if max(args) == min(args) + 1 else False

    def sample_fn(self, body, joints_limits = None):
        """
        Get the
        :param body: int
            body's ID
        :param joints_limits: list
            list of joints limits
        :return:  tuple
            sample of joints positions
        """
        if joints_limits is None:
            joints_limits = self.get_joint_limits(body, self.movable_a_joints)
        sample = [np.random.uniform(*limits) for limits in joints_limits]
        return sample

    @staticmethod
    def get_difference_fn(q1, q2):
        """
        :param q1: angle of departure
        :param q2: angle of arrival
        :return:   the difference between the angles q1 and q2
        ----------------------------------------------------------
        q(t+delta_t) = q(t) + 1/step * (q2-q1)
        """
        assert len(q1) == len(q2)
        diff = [abs(q_2 - q_1) if q_1 < q_2 else -abs(q_2 - q_1) for (q_1, q_2) in zip(q1, q2)]
        return diff

    def get_distance_fn(self, q1, q2):
        """
        Compute the distance between two angles
        :param q1: float
            angles
        :param q2: float
            angles
        :return: float
        """
        return np.sum(np.sqrt(list(map(lambda x: x ** 2, self.get_difference_fn(q1, q2)))))

    def get_extend_fn(self, q1, q2, resolutions = None):
        """
        Compute the steps angles to move from q1 to q2
        :param q1: float
            angle of departure
        :param q2: float
            angle of arrival
        :param resolutions: array
            Metric values
        :return: float
            angle
        """
        if resolutions is None:
            resolutions = 0.05 * np.ones(len(self.movable_a_joints))
        diff = self.get_difference_fn(q1, q2)

        num_steps = int(np.max(np.abs(np.divide(np.array(diff), resolutions))))

        q = q1
        for i in range(num_steps):
            q = np.array(q) + float(1.0 / num_steps) * np.array(self.get_difference_fn(q1, q2))
            yield q

    def plan_motion(self, body, goal_conf, obstacles = None, direct = True):
        """
        Plan motion to get to the end goal
        :param body: int
            body's ID
        :param goal_conf: list
            Arm angle
        :param direct: bool
            Use linear interpolation before trying RRT
        :return: list
            Path
        """
        start_conf = [state.jointPosition for state in self.get_joint_state(body, self.movable_a_joints)]
        assert len(start_conf) == len(goal_conf)
        path = []
        if direct:
            q_path = self.get_extend_fn(start_conf, goal_conf)
            for q in q_path:
                self.set_joint_positions(body, self.movable_a_joints, q)
                if self.bodies_in_collision(body, obstacles):
                    self.set_joint_positions(body, self.movable_a_joints, start_conf)
                    break
                if self.violates_limits(body, q) or self.pairs_link_in_collision(body):
                    self.set_joint_positions(body, self.movable_a_joints, start_conf)
                    break
                path.append(list(q))
            self.set_joint_positions(body, self.movable_a_joints, start_conf)
            return path
        # return self.birrt(body,start_conf,goal_conf)

    def command(self, body, path):
        """
        Send the joints in the path to the controller
        :param body: int
            body's ID
        :param path: list
            path to follow
        :param save: save the velocities
        """

        if 'vel' in self.__dict__:
            self.vel.concatenate(np.array([self.get_joint_velocities(body)]))
        for q_angles in path:
            self.joint_controller(body, q_angles)

    def joint_controller(self, body, q_angles):
        """
        Control arm
        :param body: int
            body'ID
        :param q_angles: list
            joints angles
        :param save: bool
            save the velocities
        """
        joint_positions = [state.jointPosition for state in self.get_joint_state(body, self.movable_a_joints)]

        while not np.allclose(joint_positions, q_angles, atol = 1e-3, rtol = 0):
            self.joint_control(body, self.movable_a_joints, q_angles, 'position')
            # start = time.time()
            p.stepSimulation(physicsClientId = self.client_id)
            # print ('arm: ',time.time()-start)
            if 'vel' in self.__dict__:
                self.vel.concatenate(np.array([self.get_joint_velocities(body)]))
            joint_positions = [state.jointPosition for state in self.get_joint_state(body, self.movable_a_joints)]

    def joint_control(self, body, joints = None, values = None, mode = 'position', forces = None):
        """
        Set the joint control to the corresponding control mode
        :param body: int
            body'ID
        :param joints: list
            arm joints indices
        :param values: list
            values for control mode
        :param mode: str
            control mode
        """

        if mode == 'position':
            if forces:
                return p.setJointMotorControlArray(bodyUniqueId = body,
                                                   jointIndices = joints,
                                                   controlMode = p.POSITION_CONTROL,
                                                   targetPositions = values,
                                                   forces = forces * len(joints),
                                                   physicsClientId = self.client_id)
            else:
                return p.setJointMotorControlArray(bodyUniqueId = body,
                                                   jointIndices = joints,
                                                   controlMode = p.POSITION_CONTROL,
                                                   targetPositions = values,
                                                   targetVelocities = [0.0] * len(joints),
                                                   physicsClientId = self.client_id)


        elif mode == 'velocity':
            return p.setJointMotorControlArray(bodyUniqueId = body,
                                               jointIndices = joints,
                                               controlMode = p.VELOCITY_CONTROL,
                                               targetVelocities = values,
                                               physicsClientId = self.client_id)

    def enable_gravity(self):
        """Enable gravity"""
        p.setGravity(0, 0, -9.8, physicsClientId = self.client_id)

    def disable_gravity(self):
        """Enable gravity"""
        p.setGravity(0, 0, 0, physicsClientId = self.client_id)

    def get_joint_velocities(self, body, joints = None):
        """
        Get the velocities of the joints
        :param body: int
            body's ID
        :param joints: list
            joints indices
        :return: list
            joints velocities
        """
        if joints is None:
            joints = self.movable_a_joints + self.movable_g_joints
        return [velocity.jointVelocity for velocity in self.get_joint_state(body, joints)]

    def replay_velocities(self, body, values, path = None):
        """Replay the velocities"""

        velocities = values
        joints = self.movable_a_joints + self.movable_g_joints
        self.target_vel = []
        self.joint_val = []
        n = 0
        if path:
            IMG_FOLD = os.path.abspath(os.path.join(path, 'IMG'))
            mkdir(IMG_FOLD)
            np.save(os.path.join(IMG_FOLD, 'img_%d' % n), self.get_camera_image(pixelWidth, pixelHeight))
            n += 1

            self.target_vel.append(np.array(self.get_joint_velocities(body)))
            self.joint_val.append([state.jointPosition for state in self.get_joint_state(body, joints)])

        for t in range(1, velocities.shape[0], STEP):

            self.joint_control(body, joints, velocities[t], 'velocity')
            for i in range(STEP):
                p.stepSimulation(physicsClientId = self.client_id)
            if path:
                np.save(os.path.join(IMG_FOLD, 'img_%d' % n), self.get_camera_image(pixelWidth, pixelHeight))
                n += 1
                self.target_vel.append(velocities[t])
                self.joint_val.append([state.jointPosition for state in self.get_joint_state(body, joints)])

    def gripper(self, body, mode = 'close'):
        """
        Control the gripper link
        :param body: int
            body's ID
        :param mode: str
            gripper state
        :param save: bool
            save the velocities
        """
        joints = self.movable_g_joints
        if mode == 'close':
            values = [0.5] * len(self.movable_g_joints)
        else:
            values = [0.0] * len(self.movable_g_joints)
        step = 250 if mode == 'close' else 100

        for i in range(step):
            # self.enable_gravity()
            self.joint_control(body, joints, values, 'position', [15])
            # start = time.time()
            p.stepSimulation(physicsClientId = self.client_id)
            # print ('gripper: ', time.time() - start)
            if 'vel' in self.__dict__:
                self.vel.concatenate(np.array([self.get_joint_velocities(body)]))

    def birrt(self, body, start_conf, goal_conf, iterations = 4):
        """
        BIRRT algorithm for motion planning
        :param body: int
            body's ID
        :param start_conf: list
            angles of departure
        :param goal_conf: list
            angles of arrival
        :param iterations: int
            number of iterations
        :return: list
            path with the arm configurations
        """
        node1, node2 = TreeNode(start_conf, None), TreeNode(goal_conf, None)
        route1, route2 = [node1], [node2]
        path = []
        length = []

        for _ in range(iterations):
            # Sample a random position
            q_rand = self.sample_fn(body,
                                    joints_limits = [(min(q_low, q_high), max(q_low, q_high)) for (q_low, q_high) in
                                                     zip(route1[-1].config, route2[-1].config)])

            # Nearest distance neighbor START SIDE
            distance = [self.get_distance_fn(q_rand, route_q.config) for route_q in route1]

            # Closest node edge in START SIDE
            node_parent1 = route1[distance.index(min(distance))]

            # From path start point to the random q increment by resolution
            for q in self.get_extend_fn(route1[-1].config, q_rand):
                self.set_joint_positions(body, self.movable_a_joints, q)

                if self.pairs_link_in_collision(body):
                    break
                else:
                    node_parent1 = TreeNode(list(q), parent = node_parent1)
                    route1.append(node_parent1)

            # Nearest distance neighbor END SIDE
            distance = [self.get_distance_fn(node_parent1.config
                                             , route_q.config) for route_q in route2]

            # Closest node edge in END SIDE
            node_parent2 = route2[distance.index(min(distance))]

            # From path start point to the random q increment by resolution
            for q in self.get_extend_fn(route2[-1].config, node_parent1.config):
                self.set_joint_positions(body, self.movable_a_joints, q)

                if self.pairs_link_in_collision(body):
                    break
                else:
                    node_parent2 = TreeNode(list(q), parent = node_parent2)
                    route2.append(node_parent2)

            sequences = [route.config for route in route1 + route2[::-1]]
            length.append(self.get_path_length(sequences))

            path.append(sequences)

        self.set_joint_positions(body, self.movable_a_joints, start_conf)
        return path[length.index(min(length))] if path else None

    def get_path_length(self, path):
        """
        Get the length of the path
        :param path: list
            path with arm configurations
        :return: float
            length of the path
        """
        length = 0.0
        for i in range(0, len(path) - 1, 2):
            length += self.get_distance_fn(path[i], path[i + 1])

        return length

    def set_initial_state(self, body):
        """
        Before initializing the simulation set the initial configurations of the body
        :param body: int
            body's ID
        """
        self.initial_state = self.get_joint_state(body, range(self.get_num_joints(body)))

    def reset_simulation(self, body, state = None):
        """
        Reset the simulation set
        :param body: int
            body's ID
        :param state: list
            States of the arms
        """
        if state is None:
            state = self.initial_state
        for joint_index in range(self.get_num_joints(body)):
            p.resetJointState(body,
                              jointIndex = joint_index,
                              targetValue = state[joint_index].jointPosition,
                              targetVelocity = state[joint_index].jointVelocity,
                              physicsClientId = self.client_id)

    def set_camera_pose(self, distance, yaw, pitch, target_position = np.zeros(3)):
        """
        Set the camera position
        :param distance: float
            distance from eye to camera target position
        :param yaw: float
            camera yaw angle (in degrees) left/right
        :param pitch: float
            camera pitch angle (in degrees) up/down
        :param target_position: list
            cameraTargetPosition is the camera focus point
        """
        p.resetDebugVisualizerCamera(distance, yaw, pitch, target_position, physicsClientId = self.client_id)

    def get_camera_image(self, width = 256, height = 256):
        """
        Get the rgb camera image
        :param width: int
            image width resolution
        :param height: int
            image height resolution
        :return: array(r,g,b,a)
            image rgb
        """
        # start=time.time()
        viewMatrix = p.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll,
                                                         upAxisIndex)
        aspect = width / height;
        projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
        img_arr = p.getCameraImage(width, height, viewMatrix,
                                   projectionMatrix, shadow = 1, lightDirection = [1, 1, 1],
                                   renderer = p.ER_TINY_RENDERER, physicsClientId = self.client_id)

        # print time.time() -start
        return np.array(img_arr[2], dtype = np.uint8)

    def save_body_conf(self, save = False, filename = None):
        """ Save bodies in the physics server """
        self.bodies = dict()
        for body in range(self.get_num_bodies()):
            self.bodies[body] = World(body = body,
                                      pose = self.get_position(body),
                                      joints = range(self.get_num_joints(body)),
                                      configuration = [conf.jointPosition for conf in
                                                       self.get_joint_state(body, range(self.get_num_joints(body)))])

        if save:
            position = [state.pose for state in self.bodies.values()]
            np.save(filename, np.array(position))
        return self.bodies

    def restore_body_conf(self):
        """ Restore the configurations. """
        for body, state in self.bodies.items():
            self.set_pose(body, state.pose)
            if state.configuration:
                self.set_joint_positions(body, joints = state.joints, values = state.configuration)

        if 'vel' in self.__dict__:
            joints = ['joints_%d' % i for i in self.movable_a_joints + self.movable_g_joints]
            self.vel = Pd(self.filename, columns = joints, shape = (0, len(joints)))

    def get_shape(self, body, flags = 0):
        """
        Get body's shape
        :param body: int
            body's ID
        :param flags: int
            Texture unique id
        :return: tuple
            Body shape information
        """
        return p.getVisualShapeData(body, flags, physicsClientId = self.client_id)

    def step(self, body, values, joints=None):
        """Replay the velocities"""
        velocities = values
        if joints is None:
          joints = self.movable_a_joints + self.movable_g_joints

        for t in range(0, velocities.shape[0]):
            self.joint_control(body, joints, velocities[t], 'velocity')
            for i in range(STEP):
                p.stepSimulation(physicsClientId = self.client_id)

    def step_with_gripper(self, body, vel, eef=None):
        """Replay the velocities"""
        velocities = vel
        arm_joints = self.movable_a_joints
        eef_joints = self.movable_g_joints

        self.joint_control(body, arm_joints, velocities, 'velocity')

        if eef is not None:
            if eef <0.6:
                self.joint_control(body, eef_joints, [0.0] * len(self.movable_g_joints), 'position',[15])
            else:
                self.joint_control(body, eef_joints, [0.5] * len(self.movable_g_joints), 'position',[15])
        for i in range(STEP):
            p.stepSimulation(physicsClientId = self.client_id)

    def step_with_pose(self, body, joints, poses):
        self.set_joint_positions(body, joints, poses)


def Point(x=0., y=0., z=0.):
    return np.array([x, y, z])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def Euler(roll=0., pitch=0., yaw=0.):
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler))