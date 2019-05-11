# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "rbt_baxter_msgs: 25 messages, 4 services")

set(MSG_I_FLAGS "-Irbt_baxter_msgs:/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/indigo/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(rbt_baxter_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg" "geometry_msgs/Wrench:geometry_msgs/Point:geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Twist"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg" "rbt_baxter_msgs/DigitalIOState"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv" "rbt_baxter_msgs/CameraSettings:rbt_baxter_msgs/CameraControl"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg" "rbt_baxter_msgs/CameraControl"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg" "geometry_msgs/Wrench:geometry_msgs/Point:geometry_msgs/Vector3:std_msgs/Header:geometry_msgs/Quaternion:rbt_baxter_msgs/EndpointState:geometry_msgs/Pose:geometry_msgs/Twist"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg" "rbt_baxter_msgs/NavigatorState"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg" "rbt_baxter_msgs/AnalogIOState"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg" "rbt_baxter_msgs/AssemblyState"
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg" ""
)

get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv" NAME_WE)
add_custom_target(_rbt_baxter_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "rbt_baxter_msgs" "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv" "geometry_msgs/Point:sensor_msgs/JointState:geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/PoseStamped:std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)

### Generating Services
_generate_srv_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg;/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_cpp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
)

### Generating Module File
_generate_module_cpp(rbt_baxter_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(rbt_baxter_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(rbt_baxter_msgs_generate_messages rbt_baxter_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_cpp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbt_baxter_msgs_gencpp)
add_dependencies(rbt_baxter_msgs_gencpp rbt_baxter_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbt_baxter_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)

### Generating Services
_generate_srv_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg;/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_lisp(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
)

### Generating Module File
_generate_module_lisp(rbt_baxter_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(rbt_baxter_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(rbt_baxter_msgs_generate_messages rbt_baxter_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_lisp _rbt_baxter_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbt_baxter_msgs_genlisp)
add_dependencies(rbt_baxter_msgs_genlisp rbt_baxter_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbt_baxter_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Wrench.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Twist.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_msg_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)

### Generating Services
_generate_srv_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv"
  "${MSG_I_FLAGS}"
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg;/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/indigo/share/sensor_msgs/cmake/../msg/JointState.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/indigo/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)
_generate_srv_py(rbt_baxter_msgs
  "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
)

### Generating Module File
_generate_module_py(rbt_baxter_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(rbt_baxter_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(rbt_baxter_msgs_generate_messages rbt_baxter_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/JointCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/RobustControllerStatus.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionAvoidanceState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/ListCameras.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/OpenCamera.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CollisionDetectionState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/CloseCamera.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraControl.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalOutputCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/CameraSettings.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogOutputCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndpointStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/SEAJointState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/NavigatorStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/HeadPanCommand.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/EndEffectorProperties.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AnalogIOStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/AssemblyStates.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/URDFConfiguration.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/msg/DigitalIOState.msg" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_msgs/srv/SolvePositionIK.srv" NAME_WE)
add_dependencies(rbt_baxter_msgs_generate_messages_py _rbt_baxter_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(rbt_baxter_msgs_genpy)
add_dependencies(rbt_baxter_msgs_genpy rbt_baxter_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS rbt_baxter_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/rbt_baxter_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(rbt_baxter_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(rbt_baxter_msgs_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(rbt_baxter_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/rbt_baxter_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(rbt_baxter_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(rbt_baxter_msgs_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(rbt_baxter_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/rbt_baxter_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(rbt_baxter_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(rbt_baxter_msgs_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(rbt_baxter_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
