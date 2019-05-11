# coding=utf-8

import pybullet as p
import pybullet_data
import numpy as np
import math
from collections import namedtuple
from itertools import product, combinations
from motion_planners.rrt_connect import birrt, direct_path

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
PI = np.pi
CIRCULAR_LIMITS = -PI, PI
MAX_DISTANCE = 0
CLIENT = 0

def connect():
    client_id = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId = client_id)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True, physicsClientId = client_id)

def disconnect():
    p.disconnect()

def is_connected():
    return p.getConnectionInfo(physicsClientId = CLIENT)['isConnected']

def enable_gravity():
    p.setGravity(0, 0, -9.8, physicsClientId = CLIENT)

def disable_gravity():
    p.setGravity(0, 0, 0, physicsClientId = CLIENT)

def step_simulation():
    p.stepSimulation(physicsClientId = CLIENT)

def enable_real_time():
    p.setRealTimeSimulation(1, physicsClientId = CLIENT)

def disable_real_time():
    p.setRealTimeSimulation(0, physicsClientId = CLIENT)

def set_pose(body, pose):
    (point, quat) = pose
    p.resetBasePositionAndOrientation(body, point, quat, physicsClientId=CLIENT)


def reset_simulation():
    p.resetSimulation(physicsClientId = CLIENT)

def load_model(rel_path, pose = None, fixed_base = True):
    # TODO: error with loadURDF when loading MESH visual and CYLINDER collision
    abs_path = rel_path#get_model_path(rel_path)
    flags = 0  # by default, Bullet disables self-collision
    # add_data_path()
    if abs_path.endswith('.urdf'):
        body = p.loadURDF(abs_path, useFixedBase = fixed_base, flags = flags, physicsClientId = CLIENT)
    elif abs_path.endswith('.sdf'):
        body = p.loadSDF(abs_path, physicsClientId = CLIENT)
    elif abs_path.endswith('.xml'):
        body = p.loadMJCF(abs_path, physicsClientId = CLIENT)
    elif abs_path.endswith('.bullet'):
        body = p.loadBullet(abs_path, physicsClientId = CLIENT)
    else:
        raise ValueError(abs_path)
    if pose is not None:
        set_pose(body, pose)
    # BODIES[CLIENT][body] = URDFInfo(None, abs_path)
    return body
def get_num_joints(body):
    return p.getNumJoints(body, physicsClientId=CLIENT)

def get_joints(body):
    return list(range(get_num_joints(body)))
get_links = get_joints
def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint, physicsClientId=CLIENT))


def get_joint_type(body, joint):
    return get_joint_info(body, joint).jointType

def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED

def prune_fixed_joints(body, joints):
    return [joint for joint in joints if is_movable(body, joint)]

def get_movable_joints(body): # 45 / 87 on pr2
    return prune_fixed_joints(body, get_joints(body))

def is_circular(body, joint):
    joint_info = get_joint_info(body, joint)
    if joint_info.jointType == p.JOINT_FIXED:
        return False
    return joint_info.jointUpperLimit < joint_info.jointLowerLimit

def get_sample_fn(body, joints):
    def fn():
        values = []
        for joint in joints:
            limits = CIRCULAR_LIMITS if is_circular(body, joint) \
                else get_joint_limits(body, joint)
            values.append(np.random.uniform(*limits))
        return tuple(values)
    return fn

def get_joint_limits(body, joint):
    # TODO: make a version for several joints?
    if is_circular(body, joint):
        return CIRCULAR_LIMITS
    joint_info = get_joint_info(body, joint)
    return joint_info.jointLowerLimit, joint_info.jointUpperLimit

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value, physicsClientId=CLIENT)

def inverse_kinematics(robot, link, pose, max_iterations=200, tolerance=1e-3):
    (target_point, target_quat) = pose
    movable_joints = get_movable_joints(robot)
    for iterations in range(max_iterations):
        # TODO: stop is no progress
        # TODO: stop if collision or invalid joint limits
        if target_quat is None:
            kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, physicsClientId=CLIENT)
        else:
            kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, target_quat, physicsClientId=CLIENT)
        if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
            return None
        # print('kinematic_conf', kinematic_conf)
        set_joint_positions(robot, movable_joints, kinematic_conf)
        link_point, link_quat = get_link_pose(robot, link)
        # print link_point,link_quat
        if np.allclose(link_point, target_point, atol=tolerance, rtol=0):
            # print 'found'
            break
    else:
        return None
    if violates_limits(robot, movable_joints, kinematic_conf):
        # print 'violate'
        return None
    return kinematic_conf


def violates_limit(body, joint, value):
    if not is_circular(body, joint):
        lower, upper = get_joint_limits(body, joint)
        if (value < lower) or (upper < value):
            return True
    return False

def violates_limits(body, joints, values):
    return any(violates_limit(body, joint, value) for joint, value in zip(joints, values))




def get_pose(body):
    return p.getBasePositionAndOrientation(body, physicsClientId=CLIENT)

def get_link_pose(body, link):
    if link == -1:
        return get_pose(body)
    # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
    link_state = get_link_state(body, link)
    return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

def get_link_state(body, link):
    return LinkState(*p.getLinkState(body, link, physicsClientId=CLIENT))

def pairwise_collision(body1, body2, max_distance=MAX_DISTANCE): # 10000
    # TODO: confirm that this doesn't just check the base link
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  physicsClientId=CLIENT)) != 0 # getContactPoints



def get_ik_fn(robot, fixed = [], teleport = False, num_attempts = 10):
    movable_joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, movable_joints)

    def fn(body, eef_link,target_pose):
        # obstacles = [body] + fixed
        obstacles = []

        for _ in range(num_attempts):
            # set_joint_positions(robot, movable_joints, sample_fn())  # Random seed
            # TODO: multiple attempts?
            q_approach = inverse_kinematics(robot, eef_link, target_pose)

            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                continue

            if q_approach:
                break
            # conf = BodyConf(robot, q_approach)
            # q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)
            # if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
            #     continue
            # if teleport:
            #     path = [q_approach, q_grasp]
            # else:
            #     conf.assign()
            #     # direction, _ = grasp.approach_pose
            #     # path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
            #     #                                   quat_from_pose(approach_pose))
            #     path = plan_joint_motion(robot, conf.joints, q_grasp, obstacles = obstacles, direct = True)
            #     if path is None:
            #         if DEBUG_FAILURE: user_input('Approach motion failed')
            #         continue
            # command = Command([BodyPath(robot, path),
            #                    Attach(body, robot, grasp.link),
            #                    BodyPath(robot, path[::-1], attachments = [grasp])])
            # return (conf, command)

        return q_approach
            # TODO: holding collisions


    return fn

def get_free_motion_gen(robot, fixed=[], teleport=False):
    def fn(conf1, conf2, fluents=[]):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            # obstacles = fixed + assign_fluent_state(fluents)
            obstacles = fixed
            path = plan_joint_motion(robot,conf2.joints, conf2.configuration, obstacles=obstacles)
        #     if path is None:
        #         if DEBUG_FAILURE: user_input('Free motion failed')
        #         return None
        # command = Command([BodyPath(robot, path)])
        # return (command,)
        return path
    return fn

def plan_joint_motion(body, joints, end_conf, obstacles=None, attachments=[],
                      self_collisions=True, disabled_collisions=set(), direct=False,
                      weights=None, resolutions=None, **kwargs):
    # if direct:
    #     return plan_direct_joint_motion(body, joints, end_conf, obstacles, attachments, self_collisions, disabled_collisions)
    assert len(joints) == len(end_conf)
    sample_fn = get_sample_fn(body, joints)
    distance_fn = get_distance_fn(body, joints, weights=weights)
    extend_fn = get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions)

    start_conf = get_joint_positions(body, joints)
    if not check_initial_end(start_conf, end_conf, collision_fn):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)

def check_initial_end(start_conf, end_conf, collision_fn):
    if collision_fn(start_conf):
        print("Warning: initial configuration is in collision")
        return False
    if collision_fn(end_conf):
        print("Warning: end configuration is in collision")
        return False
    return True

def get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions, use_limits=True):
    check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) if self_collisions else []
    moving_bodies = [body] + [attachment.child for attachment in attachments]
    if obstacles is None:
        obstacles = list(set(get_bodies()) - set(moving_bodies))
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    # TODO: maybe prune the link adjacent to the robot
    # TODO: test self collision with the holding
    # TODO: end-effector constraints
    def collision_fn(q):
        if use_limits and violates_limits(body, joints, q):
            return True
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        for link1, link2 in check_link_pairs:
            if pairwise_link_collision(body, link1, body, link2):
                return True
        return any(pairwise_collision(*pair) for pair in check_body_pairs)
    return collision_fn

def get_bodies():
    return [p.getBodyUniqueId(i, physicsClientId=CLIENT)
            for i in range(p.getNumBodies(physicsClientId=CLIENT))]

def pairwise_link_collision(body1, link1, body2, link2, max_distance=MAX_DISTANCE): # 10000
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  linkIndexA=link1, linkIndexB=link2,
                                  physicsClientId=CLIENT)) != 0

def get_all_link_parents(body):
    return {link: get_link_parent(body, link) for link in get_links(body)}

def get_all_link_children(body):
    children = {}
    for child, parent in get_all_link_parents(body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children

def get_link_children(body, link):
    children = get_all_link_children(body)
    return children.get(link, [])

def get_link_descendants(body, link):
    descendants = []
    for child in get_link_children(body, link):
        descendants.append(child)
        descendants += get_link_descendants(body, child)
    return descendants

def get_moving_links(body, moving_joints):
    moving_links = list(moving_joints)
    for link in moving_joints:
        moving_links += get_link_descendants(body, link)
    return list(set(moving_links))

def get_link_ancestors(body, link):
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]

def get_joint_ancestors(body, link):
    return get_link_ancestors(body, link) + [link]


def get_moving_pairs(body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = get_moving_links(body, moving_joints)
    for i in range(len(moving_links)):
        link1 = moving_links[i]
        ancestors1 = set(get_joint_ancestors(body, link1)) & set(moving_joints)
        for j in range(i+1, len(moving_links)):
            link2 = moving_links[j]
            ancestors2 = set(get_joint_ancestors(body, link2)) & set(moving_joints)
            if ancestors1 != ancestors2:
                yield link1, link2

def get_self_link_pairs(body, joints, disabled_collisions=set()):
    moving_links = get_moving_links(body, joints)
    fixed_links = list(set(get_links(body)) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if True:
        check_link_pairs += list(get_moving_pairs(body, joints))
    else:
        check_link_pairs += list(combinations(moving_links, 2))
    check_link_pairs = list(filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                                (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs

def are_links_adjacent(body, link1, link2):
    return (get_link_parent(body, link1) == link2) or \
           (get_link_parent(body, link2) == link1)

def get_extend_fn(body, joints, resolutions=None):
    if resolutions is None:
        resolutions = 0.05*np.ones(len(joints))
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        refine_fn = get_refine_fn(body, joints, num_steps=int(np.max(steps)))
        return refine_fn(q1, q2)
    return fn

def get_refine_fn(body, joints, num_steps=0):
    difference_fn = get_difference_fn(body, joints)
    num_steps = num_steps + 1
    def fn(q1, q2):
        q = q1
        for i in range(num_steps):
            q = tuple((1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints
    return fn

def get_distance_fn(body, joints, weights=None):
    # TODO: use the energy resulting from the mass matrix here?
    if weights is None:
        weights = 1*np.ones(len(joints))
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return fn

def get_difference_fn(body, joints):
    def fn(q2, q1):
        difference = []
        for joint, value2, value1 in zip(joints, q2, q1):
            difference.append(circular_difference(value2, value1)
                              if is_circular(body, joint) else (value2 - value1))
        return tuple(difference)
    return fn


def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

def circular_difference(theta2, theta1):
    return wrap_angle(theta2 - theta1)

def get_link_name(body, link):
    if link == -1:
        return get_base_name(body)
    return get_joint_info(body, link).linkName

def get_link_parent(body, link):
    if link == -1:
        return None
    return get_joint_info(body, link).parentIndex

def link_from_name(body, name):
    if name == get_base_name(body):
        return -1
    for link in get_joints(body):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])
def get_body_info(body):
    return BodyInfo(*p.getBodyInfo(body, physicsClientId=CLIENT))

def get_base_name(body):
    return get_body_info(body).base_name.decode(encoding='UTF-8')


def get_joint_state(body, joint):
    return JointState(*p.getJointState(body, joint, physicsClientId=CLIENT))


def get_configuration(body):
    return get_joint_positions(body, get_movable_joints(body))

def get_joint_positions(body, joints): # joints=None):
    return tuple(get_joint_position(body, joint) for joint in joints)

def get_joint_position(body, joint):
    return get_joint_state(body, joint).jointPosition

class BodyConf(object):
    def __init__(self, body, configuration=None, joints=None):
        if joints is None:
            joints = get_movable_joints(body)
        if configuration is None:
            configuration = get_configuration(body)
        self.body = body
        self.joints = joints
        self.configuration = configuration
    def assign(self):
        set_joint_positions(self.body, self.joints, self.configuration)
        return self.configuration
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)