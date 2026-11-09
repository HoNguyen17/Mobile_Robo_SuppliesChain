cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_control)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/astar_planner.py
  scripts/dstar_planner.py
  scripts/path_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Modules that the planners and path_follower.py import (astar_planner.py also imports
# dstar_lite.py). They must sit next to the scripts.
install(FILES
  scripts/dstar_lite.py
  scripts/grid_planner.py
  scripts/nav_common.py
  scripts/path_tracker.py
  scripts/plan_scheduler.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
