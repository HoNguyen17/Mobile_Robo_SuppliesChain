cmake_minimum_required(VERSION 3.0.2)
project(warehouse_mission)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs
)

catkin_install_python(PROGRAMS
  scripts/mission_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
