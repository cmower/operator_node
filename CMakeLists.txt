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
  message_generation
  )

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(isometric_node src/isometric_node.cpp)
target_link_libraries(isometric_node ${catkin_LIBRARIES})

add_executable(joy_remap_node src/joy_remap.cpp)
target_link_libraries(joy_remap_node ${catkin_LIBRARIES})

add_executable(interface_logger_node src/operator_interface_logger.cpp)
target_link_libraries(interface_logger_node ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/scale_node.py
  scripts/mouse_input_node.py
  scripts/keyboard_to_joy.py
  scripts/Campeau2019Intuitive_node.py
  scripts/teleop_tf_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
