cmake_minimum_required(VERSION 2.8.3)
project(wall_follow_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  msgs
)

catkin_package()

SET(CMAKE_BUILD_TYPE Debug)

# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(wall_follow_control_node src/pid_controller.cpp src/pid_controller.h src/wall_follow_control_node.cpp )

target_link_libraries(wall_follow_control_node
  ${catkin_LIBRARIES}
)
