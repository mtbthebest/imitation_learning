# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bot_gazebo: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bot_gazebo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv" NAME_WE)
add_custom_target(_bot_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bot_gazebo" "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(bot_gazebo
  "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bot_gazebo
)

### Generating Module File
_generate_module_cpp(bot_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bot_gazebo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bot_gazebo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bot_gazebo_generate_messages bot_gazebo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv" NAME_WE)
add_dependencies(bot_gazebo_generate_messages_cpp _bot_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bot_gazebo_gencpp)
add_dependencies(bot_gazebo_gencpp bot_gazebo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bot_gazebo_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(bot_gazebo
  "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bot_gazebo
)

### Generating Module File
_generate_module_lisp(bot_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bot_gazebo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bot_gazebo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bot_gazebo_generate_messages bot_gazebo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv" NAME_WE)
add_dependencies(bot_gazebo_generate_messages_lisp _bot_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bot_gazebo_genlisp)
add_dependencies(bot_gazebo_genlisp bot_gazebo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bot_gazebo_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(bot_gazebo
  "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bot_gazebo
)

### Generating Module File
_generate_module_py(bot_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bot_gazebo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bot_gazebo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bot_gazebo_generate_messages bot_gazebo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/mtb/sim_ws/bot_ws/src/fetch_pkg/bot_gazebo/srv/ImageState.srv" NAME_WE)
add_dependencies(bot_gazebo_generate_messages_py _bot_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bot_gazebo_genpy)
add_dependencies(bot_gazebo_genpy bot_gazebo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bot_gazebo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bot_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bot_gazebo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bot_gazebo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bot_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bot_gazebo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bot_gazebo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bot_gazebo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bot_gazebo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bot_gazebo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bot_gazebo_generate_messages_py std_msgs_generate_messages_py)
endif()
