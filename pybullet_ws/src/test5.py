# coding=utf-8

from pblt import Interface,MOVABLE_ARM_JOINTS
import time

from geometry import Point,Pose,Euler

ROBOT_URDF_PATH ='../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'
TABLE = '../models/table/table.urdf'
CUBE_URDF = '../models/cube/%s_cube.urdf'
BASKET_URDF = '../models/basket/basket.urdf'


with Interface() as I:

    robot= I.load_model(ROBOT_URDF_PATH)
    table = I.load_model(TABLE, Pose(Point(0.35, 0.0, 0.05), Euler(0.0, 0.0, 1.57)), fixed_base = False, scaling = 0.3)
    redBlock1 = I.load_model(CUBE_URDF % 'red', Pose(Point(0.22, -0.2, 0.265)), fixed_base = False)


    basket =  I.load_model(BASKET_URDF, Pose(Point(0.35, 0.05, 0.265)), fixed_base = False)

    def move_to(target_pose,gripper_link):
        paths = []
        for poses in target_pose:
            end_conf = I.calculate_inverse_kinematics(robot,gripper_link,poses)

            if end_conf is not None:
                path = I.plan_motion(robot, end_conf)
                if path is None:
                    return None
                else:
                    paths.append(path)
            else: return  None
            I.set_joint_positions(robot,MOVABLE_ARM_JOINTS,end_conf)
        return paths

    def stack(paths):
        I.command(robot, paths[0],save=True)
        I.command(robot, paths[1],save=True)
        I.gripper(robot, 'close',save=True)
        I.command(robot, paths[2],save=True)
        I.command(robot, paths[3],save=True)
        # I.command(robot, paths[4],save=True)
        I.gripper(robot, 'open',save=True)
        # I.command(robot, paths[4],save=True)


    if __name__ == '__main__':


        gripper_link = I.get_link_from_name(robot,END_EFFECTOR_LINK)
        eef_state = I.get_link_state(robot,gripper_link)
        eef_pose,eef_orientation = eef_state.worldLinkFramePosition,I.get_euler_from_quaternion(eef_state.worldLinkFrameOrientation)

        initial_target = Pose(Point(0.35,0.0,0.5),Euler(0.0,3.14,0.0))



        #limits
        x =[ 0.25,-0.25]
        y =[ 0.25,-0.25]
        target_pose1 = Pose(Point(0.22, -0.2, 0.4), Euler(0.0, 3.14, 0.0))
        target_pose2 = Pose(Point(0.22, -0.2, 0.285), Euler(0.0, 3.14, 0.0))
        target_pose3 = Pose(Point(0.35, 0.05, 0.4), Euler(0.0, 3.14, 0.0))
        # target_pose4 = Pose(Point(0.35, 0.05, 0.2), Euler(0.0, 3.14, 0.0))

        I.set_initial_state(robot)
        paths = move_to([target_pose1,target_pose2,initial_target,target_pose3,initial_target],gripper_link)

        I.reset_simulation(robot)
        stack(paths)

        time.sleep(5.0)

