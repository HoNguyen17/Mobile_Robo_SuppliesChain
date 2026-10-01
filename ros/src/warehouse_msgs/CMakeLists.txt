cmake_minimum_required(VERSION 3.0.2)
project(warehouse_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  FILES
  Task.msg
  CollisionEvent.msg
  RunEvent.msg
  RunSummary.msg
)

add_service_files(
  FILES
  AttachItem.srv
  ReleaseItem.srv
  ResetScenario.srv
  GetNextTask.srv
  ReportRun.srv
  SetCategory.srv
  StartRun.srv
  MarkLeg.srv
  FinishRun.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
)
