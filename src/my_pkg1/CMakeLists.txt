cmake_minimum_required(VERSION 3.0.2)
project(my_pkg1)


find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  pcl_ros

  visualization_msgs
  geometry_msgs

)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/filter_by_height.py
  scripts/flatten_slice_by_height.py
  scripts/nearest_piont.py
  scripts/basket_detector_node.py
  scripts/line_remover_node.py
  scripts/test1.py
  scripts/test2.py
  scripts/test3.py
  scripts/test4.py
  scripts/test5.py
  scripts/test6.py
  scripts/test7.py
  scripts/cam_test1.py

  scripts/circle_strong.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

