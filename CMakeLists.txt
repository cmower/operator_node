cmake_minimum_required(VERSION 3.0.2)
project(operator_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  joy
  tf2
  tf2_ros
  keyboard
  custom_srvs
  )

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(joy_remap_node src/joy_remap.cpp)
target_link_libraries(joy_remap_node ${catkin_LIBRARIES})

add_executable(interface_logger_node src/operator_interface_logger.cpp)
target_link_libraries(interface_logger_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/scale_node.py
  scripts/keyboard_to_joy.py
  scripts/isometric_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
