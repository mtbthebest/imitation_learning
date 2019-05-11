cmake_minimum_required(VERSION 2.8.3)
project(rbt_baxter_hardware)

find_package(catkin
  REQUIRED COMPONENTS
  std_msgs
  roscpp
  rbt_baxter_msgs
  cv_bridge
  image_transport
  rbt_baxter_kinematics
  tf
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(cmake_modules REQUIRED)
find_package(OpenCV 2.4.8 REQUIRED)

find_package(Eigen REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    rbt_baxter_msgs
    cv_bridge
    image_transport
    rbt_baxter_kinematics
  DEPENDS OpenCV
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(rbt_baxter_emulator src/rbt_baxter_emulator.cpp)
target_link_libraries(rbt_baxter_emulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(rbt_baxter_emulator ${catkin_EXPORTED_TARGETS})

install(
  TARGETS rbt_baxter_emulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(
  DIRECTORY config images include launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
