cmake_minimum_required(VERSION 3.8)
project(cell_planner_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanningReport.msg"
  "msg/ExecutorFact.msg"
  "msg/Ruling.msg"
  "msg/WatchdogNotice.msg"
  "msg/PhysicalEvent.msg"
  "srv/PlanToPose.srv"
  "srv/PlanToJoints.srv"
  "srv/PlanCartesian.srv"
  "srv/CheckTrajectory.srv"
  "srv/AddCollisionObject.srv"
  "srv/RemoveCollisionObject.srv"
  "srv/SyncCollisionObjects.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs trajectory_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
