cmake_minimum_required(VERSION 3.8)
project(rviz2_cinematographer_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Define message dependencies
set(MSG_DEPS
  std_msgs
  geometry_msgs
)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraMovement.msg"
  "msg/CameraTrajectory.msg"
  "msg/Record.msg"
  "msg/Finished.msg"
  "msg/Wait.msg"
  DEPENDENCIES ${MSG_DEPS}
)

# Export package
ament_export_dependencies(rosidl_default_runtime ${MSG_DEPS})
ament_package()
