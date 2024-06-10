cmake_minimum_required(VERSION 3.8)
project(blueye_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToPose.action"
  "action/Homing.action"
  "action/AdjustingPosition.action"
  "action/InitializeDocking.action"
  "action/NavigateWaypoints.action"
  "action/Maneuvering.action"
  "msg/DVL.msg"
  "msg/DesiredVelocity.msg"
  "msg/DesiredForces.msg"
  "msg/Kalman.msg"
  "srv/AddWaypoint.srv"
  "srv/ClearWaypoints.srv"
  "srv/PrintWaypoints.srv"
  "srv/RemoveWaypoint.srv"
  "srv/SetState.srv"
  DEPENDENCIES builtin_interfaces
  DEPENDENCIES std_msgs
  DEPENDENCIES geometry_msgs
  DEPENDENCIES action_msgs
)

ament_package()
