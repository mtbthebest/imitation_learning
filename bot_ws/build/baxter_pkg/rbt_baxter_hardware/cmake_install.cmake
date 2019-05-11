# Install script for directory: /home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_hardware

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/mtb/sim_ws/bot_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/mtb/sim_ws/bot_ws/build/baxter_pkg/rbt_baxter_hardware/catkin_generated/installspace/rbt_baxter_hardware.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rbt_baxter_hardware/cmake" TYPE FILE FILES
    "/home/mtb/sim_ws/bot_ws/build/baxter_pkg/rbt_baxter_hardware/catkin_generated/installspace/rbt_baxter_hardwareConfig.cmake"
    "/home/mtb/sim_ws/bot_ws/build/baxter_pkg/rbt_baxter_hardware/catkin_generated/installspace/rbt_baxter_hardwareConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rbt_baxter_hardware" TYPE FILE FILES "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_hardware/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware" TYPE EXECUTABLE FILES "/home/mtb/sim_ws/bot_ws/devel/lib/rbt_baxter_hardware/rbt_baxter_emulator")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator"
         OLD_RPATH "/opt/ros/indigo/lib:/home/mtb/sim_ws/bot_ws/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/rbt_baxter_hardware/rbt_baxter_emulator")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rbt_baxter_hardware" TYPE DIRECTORY FILES
    "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_hardware/config"
    "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_hardware/images"
    "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_hardware/include"
    "/home/mtb/sim_ws/bot_ws/src/baxter_pkg/rbt_baxter_hardware/launch"
    )
endif()

