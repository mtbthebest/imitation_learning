cmake_minimum_required(VERSION 2.8.3)
project(rbt_baxter_gazebo)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  gazebo_ros_control
  controller_manager
  rbt_baxter_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  rbt_baxter_msgs
  gazebo_ros_control
  roscpp
  controller_manager_msgs
  DEPENDS
  gazebo
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(rbt_baxter_gazebo_ros_control
  src/rbt_baxter_gazebo_ros_control_plugin.cpp
)

target_link_libraries(rbt_baxter_gazebo_ros_control
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  gazebo_ros_control
)

add_dependencies(
  rbt_baxter_gazebo_ros_control
  ${catkin_EXPORTED_TARGETS}
)

install(
  TARGETS rbt_baxter_gazebo_ros_control
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)