cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_control)

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
  scripts/astar_planner.py
  scripts/path_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Modules that astar_planner.py and path_follower.py import. They must sit next to the scripts.
install(FILES
  scripts/grid_planner.py
  scripts/nav_common.py
  scripts/path_tracker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
