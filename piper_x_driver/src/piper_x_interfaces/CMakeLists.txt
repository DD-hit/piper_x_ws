cmake_minimum_required(VERSION 3.8)
project(piper_x_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArmStatus.msg"
  "msg/MotionResult.msg"
  "msg/MotionFeedback.msg"
  "action/MoveJ.action"
  "action/MoveP.action"
  "action/MoveL.action"
  "action/MoveC.action"
  "action/MoveT.action"
  "action/SetGripper.action"
  "action/RunTask.action"
  DEPENDENCIES action_msgs geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
