execute_process(COMMAND "/home/rover-avvb/catkin_ws/build/ros_imu_bno055/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/rover-avvb/catkin_ws/build/ros_imu_bno055/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
