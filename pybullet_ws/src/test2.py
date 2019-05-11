# coding=utf-8
from pblt import Interface
import time
from geometry import Point,Pose,Euler

MOVABLE_ARM_JOINTS = [1,2,3,4,5,6]
ROBOT_URDF_PATH ='../models/ur5_description/urdf/ur5.urdf'
END_EFFECTOR_LINK = 'end_effector_link'
TABLE = '../models/table/table.urdf'
CUBE_URDF = '../models/cube/%s_cube.urdf'

I=Interface(save = True)
I.connect()


x_offset = 0.33
y_offset = 0.15

robot= I.load_model(ROBOT_URDF_PATH)

redBlock1 = I.load_model(CUBE_URDF%'red',Pose(Point(0.3+x_offset,-0.1+y_offset,0.265)),fixed_base=False)
redBlock2 = I.load_model(CUBE_URDF%'red',Pose(Point(0.35+x_offset,0.05+y_offset,0.265)),fixed_base=False)
table = I.load_model(TABLE,Pose(Point(0.35+x_offset,0.0+y_offset,0.05),Euler(0.0,0.0,1.57)),fixed_base=False,scaling=0.3)


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
    save = True
    I.command(robot, paths[0],save)
    I.command(robot, paths[1], save)
    I.gripper(robot, 'close',save)
    I.command(robot, paths[2],save)
    I.command(robot, paths[3],save)
    I.gripper(robot, 'open',save)

if __name__ == '__main__':


    gripper_link = I.get_link_from_name(robot,END_EFFECTOR_LINK)
    eef_state = I.get_link_state(robot,gripper_link)
    eef_pose,eef_orientation = eef_state.worldLinkFramePosition,I.get_euler_from_quaternion(eef_state.worldLinkFrameOrientation)

    initial_target = Pose(Point(0.35,0.0,0.5),Euler(0.0,1.57,0.0))

    target_pose1 = Pose(Point(0.3+x_offset,-0.1+y_offset,0.5), Euler(0.0, 1.57, 0.0))
    target_pose2 = Pose(Point(0.3+x_offset,-0.1+y_offset,0.275), Euler(0.0, 1.57, 0.0))
    target_pose3 = Pose(Point(0.35+x_offset,0.05+y_offset,0.35), Euler(0.0, 1.57, 0.0))

    I.set_initial_state(robot)

    paths = move_to([target_pose1,target_pose2,target_pose1,target_pose3],gripper_link)
    I.reset_simulation(robot)
    time.sleep(2.0)
    stack(paths)

I.disconnect()