# coding=utf-8

from pblt import Interface, MOVABLE_ARM_JOINTS
import time

from geometry import Point, Pose, Euler

ROBOT_URDF_PATH = '../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
BLOCK_URDF = "../ss-pybullet-master/models/drake/objects/block_for_pick_and_place_mid_size.urdf"
END_EFFECTOR_LINK = 'jaco_eef_link'
TABLE = '../models/cube_table/cube_table.urdf'
CUBE_URDF = '../models/cube/%s_cube.urdf'
TRAY_URDF = '../models/tray/tray.urdf'
DISABLE_COLLISIONS = [
    {'link1': "shoulder_link", 'link2': "upper_arm_link"},
    {"link1": "wrist_1_link", 'link2': "wrist_2_link"},
    {'link1': "wrist_1_link", 'link2': "wrist_3_link"},
    {'link1': "wrist_2_link", 'link2': "wrist_3_link"},
    {'link1': "base_link", 'link2': "shoulder_link"},
    {'link1': "base_link", 'link2': "upper_arm_link"}
]
z_height = 0.126
x_min= 0.2
x_max= 0.35
y_min = -0.3
y_max= 0.3
with Interface() as I:
    robot = I.load_model(ROBOT_URDF_PATH)

    # I.set_camera_pose(0.6, 155, -50, [0.2, 0.0, 0.5])

    redBlock1 = I.load_model(CUBE_URDF % 'red', Pose(Point(x_min, y_min, z_height)), fixed_base = False)
    redBlock2 = I.load_model(CUBE_URDF % 'red', Pose(Point(x_max + 0.1, y_max-0.1, z_height)), fixed_base = False)
    # greenBlock1 = I.load_model(CUBE_URDF % 'green', Pose(Point(0.38, -0.1, 0.2599)), fixed_base = False)
    # greenBlock2 = I.load_model(CUBE_URDF % 'green', Pose(Point(0.38, 0.2, 0.2599)), fixed_base = False)
    table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)), fixed_base = True, scaling = 1.0)

    time.sleep(5.0)

    # img = I.get_camera_image(480, 640)


    def move_to(target_pose, gripper_link):
        paths = []
        for poses in target_pose:
            end_conf = I.calculate_inverse_kinematics(robot, gripper_link, poses)

            if end_conf is not None:
                path = I.plan_motion(robot, end_conf,[2,3,4,5])
                if path is None:
                    return None
                else:
                    paths.append(path)
            else:
                return None
            I.set_joint_positions(robot, MOVABLE_ARM_JOINTS, end_conf)
        return paths


    def stack(paths):
        I.command(robot, paths[0], save = True)
        I.command(robot, paths[1], save = True)
        I.gripper(robot, 'close', save = True)
        I.command(robot, paths[2], save = True)
        # I.command(robot, paths[3], save = True)
        # I.command(robot, paths[4], save = True)
        # I.gripper(robot, 'open', save = True)
        # I.command(robot, paths[5], save = True)


    if __name__ == '__main__':
        gripper_link = I.get_link_from_name(robot, END_EFFECTOR_LINK)
        eef_state = I.get_link_state(robot, gripper_link)
        eef_pose, eef_orientation = eef_state.worldLinkFramePosition, I.get_euler_from_quaternion(
            eef_state.worldLinkFrameOrientation)

        initial_target = Pose(Point(0.35, 0.0, 0.5), Euler(0.0, 3.14, 0.0))

        # limits
        x = [0.25, -0.25]
        y = [0.25, -0.25]
        target_pose1 = Pose(Point(x_min, y_min, z_height + 0.2), Euler(0.0, 3.14, 0.0))
        target_pose2 = Pose(Point(x_min, y_min, z_height + 0.01), Euler(0.0, 3.14, 0.0))
        # target_pose3 = Pose(Point(0.38, 0.2, 0.45), Euler(0.0, 3.14, 0.0))
        # target_pose4 = Pose(Point(0.38, 0.2, 0.365), Euler(0.0, 3.14, 0.0))

        I.set_initial_state(robot)
        paths = move_to([target_pose1,target_pose2,initial_target],
                        gripper_link)

        I.reset_simulation(robot)
        stack(paths)

    time.sleep(5.0)
