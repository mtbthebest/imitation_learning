#!/usr/bin/env python

import rospy
from bot_gazebo.srv import ImageState


# def set_states(**kwargs):
# 	# rospy.init_node('model_state_client')
# 	rospy.wait_for_service('/gazebo/set_model_state')
# 	model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
# 	for name, pose in kwargs.items():
# 		model_state = ModelState()
# 		model_state.model_name = name
# 		model_state.pose = Pose(Point(*pose), Quaternion(*(0.0, 0.0, 0.0, 0.0)))
# 		model_state.twist = Twist(Vector3(*(0.0, 0.0, 0.0)), Vector3(*(0.0, 0.0, 0.0)))
# 		model_state.reference_frame = ''
# 		model_state_service(model_state)
# 		rospy.sleep(1.0)

if __name__ == '__main__':
	rospy.init_node('viison_client')
	rospy.wait_for_service('state', timeout = 10.0)
	client = rospy.ServiceProxy('state', ImageState)
	state = client()
	print(state)