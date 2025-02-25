cmake_minimum_required(VERSION 3.8)
project(limo_gps_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Declare the service files to be built
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetNoiseParams.srv"
  DEPENDENCIES builtin_interfaces
)

ament_package()
