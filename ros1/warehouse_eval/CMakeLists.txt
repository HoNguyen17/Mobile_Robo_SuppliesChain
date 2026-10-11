cmake_minimum_required(VERSION 3.0.2)
project(warehouse_eval)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  warehouse_mission
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs warehouse_mission
)

catkin_install_python(PROGRAMS
  scripts/run_scenario.py
  scripts/run_all_scenarios.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

