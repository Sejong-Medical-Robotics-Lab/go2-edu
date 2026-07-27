cmake_minimum_required(VERSION 3.8)
project(go2_edu_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/IMUState.msg"
  "msg/LowState.msg"
  "msg/SportModeState.msg"
  "msg/ApiCommand.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
