cmake_minimum_required(VERSION 3.0.2)
project(operator_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  joy
  keyboard
  )

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(joy_remap src/joy_remap.cpp)
target_link_libraries(joy_remap ${catkin_LIBRARIES})

add_executable(operator_node_two_axis src/operator_node_two_axis.cpp)
target_link_libraries(operator_node_two_axis ${catkin_LIBRARIES})

add_executable(keyboard_to_joy src/keyboard_to_joy.cpp)
target_link_libraries(keyboard_to_joy ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/operator_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
