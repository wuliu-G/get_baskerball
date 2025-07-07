execute_process(COMMAND "/home/r2/test_G/build/my_pkg1/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/r2/test_G/build/my_pkg1/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
