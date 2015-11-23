cmake_minimum_required(VERSION 2.8.3)
project(to_cell_behavior)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  test_server
  mission
  line_pid
  relative_move_server
)


catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
# add_executable(relative_move_server_node src/relative_move_server_node.cpp)
add_executable(to_cell_behavior_node src/to_cell_behavior_node.cpp)
add_dependencies(to_cell_behavior_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(to_cell_behavior_node
  ${catkin_LIBRARIES}
)

# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
