execute_process(COMMAND "/home/mtb/sim_ws/bot_ws/build/baxter_pkg/rbt_baxter_interface/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/mtb/sim_ws/bot_ws/build/baxter_pkg/rbt_baxter_interface/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
