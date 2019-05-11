# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "jaco_vrep_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ijaco_vrep_msgs:/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(jaco_vrep_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg" NAME_WE)
add_custom_target(_jaco_vrep_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "jaco_vrep_msgs" "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(jaco_vrep_msgs
  "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaco_vrep_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(jaco_vrep_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaco_vrep_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(jaco_vrep_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(jaco_vrep_msgs_generate_messages jaco_vrep_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg" NAME_WE)
add_dependencies(jaco_vrep_msgs_generate_messages_cpp _jaco_vrep_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaco_vrep_msgs_gencpp)
add_dependencies(jaco_vrep_msgs_gencpp jaco_vrep_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaco_vrep_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(jaco_vrep_msgs
  "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaco_vrep_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(jaco_vrep_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaco_vrep_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(jaco_vrep_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(jaco_vrep_msgs_generate_messages jaco_vrep_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg" NAME_WE)
add_dependencies(jaco_vrep_msgs_generate_messages_lisp _jaco_vrep_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaco_vrep_msgs_genlisp)
add_dependencies(jaco_vrep_msgs_genlisp jaco_vrep_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaco_vrep_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(jaco_vrep_msgs
  "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaco_vrep_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(jaco_vrep_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaco_vrep_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(jaco_vrep_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(jaco_vrep_msgs_generate_messages jaco_vrep_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/jaco_pkg/jaco_vrep_msgs/msg/Torques.msg" NAME_WE)
add_dependencies(jaco_vrep_msgs_generate_messages_py _jaco_vrep_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(jaco_vrep_msgs_genpy)
add_dependencies(jaco_vrep_msgs_genpy jaco_vrep_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS jaco_vrep_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaco_vrep_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/jaco_vrep_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(jaco_vrep_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaco_vrep_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/jaco_vrep_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(jaco_vrep_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaco_vrep_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaco_vrep_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/jaco_vrep_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(jaco_vrep_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
