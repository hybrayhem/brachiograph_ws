cmake_minimum_required(VERSION 3.5)
project(brachiograph_simulation)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(joint_controller src/joint_controller.cpp)
ament_target_dependencies(joint_controller rclcpp std_msgs)

install(TARGETS
  joint_controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()