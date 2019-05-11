#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from geometry_msgs.msg import Point, Quaternion, Pose, Twist, Vector3


def get_states(request):
	model_state = request.model_state
	model_state.model_name = 'cube_green_1'
	model_state.pose = Pose(Point(*(1.5, 0.0, 0.83)), Quaternion(*(0.0, 0.0, 0.0, 0.0)))
	model_state.twist = Twist(Vector3(*(0.0, 0.0, 0.0)), Vector3(*(0.0, 0.0, 0.0)))
	model_state.reference_frame = ''
	return SetModelStateResponse(True, 'Done')


if __name__ == '__main__':
	rospy.init_node('model_state_server')
	model_state_service = rospy.Service('/gazebo/set_model_state', SetModelState, get_states)
	rospy.spin()
