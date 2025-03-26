cmake_minimum_required(VERSION 3.5)
project(arc_detector_node)

# Message generation
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PartialArc.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Dependencies for running Python node
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

