cmake_minimum_required(VERSION 3.5)
project(turtle_interfaces)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(turtlesim REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GeneratePOI.srv"
  DEPENDENCIES turtlesim
)

ament_package()
