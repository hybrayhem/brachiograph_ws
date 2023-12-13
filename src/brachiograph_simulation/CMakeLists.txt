cmake_minimum_required(VERSION 2.8.3)
project(brachiograph_simulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(joint_controller src/joint_controller.cpp)
target_link_libraries(joint_controller ${catkin_LIBRARIES})