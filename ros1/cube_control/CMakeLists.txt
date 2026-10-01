cmake_minimum_required(VERSION 3.0.2)
project(cube_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/hello_talker.py
  scripts/pose_listener.py
  scripts/cmd_vel_publisher.py
  scripts/go_to_goal.py
  scripts/map_viewer.py
  scripts/astar_planner.py
  scripts/path_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
