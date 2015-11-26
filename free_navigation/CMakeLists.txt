cmake_minimum_required(VERSION 2.8.3)
project(free_navigation)

find_package(catkin REQUIRED COMPONENTS
  mission
  msgs
  roscpp
  rospy
  roslaunch
  tf
  actionlib_msgs
  actionlib
  dock_with_tape
  relative_move_server
)

catkin_python_setup()

add_action_files(DIRECTORY action FILES NavigateFreely.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS mission msgs roscpp roslaunch tf
  DEPENDS message_runtime
)

roslaunch_add_file_check(launch)

# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(free_navigation_node
  src/free_navigation.h
  src/free_navigation_node.cpp
  src/free_navigation.cpp
)

add_dependencies(free_navigation_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(free_navigation_node
  ${catkin_LIBRARIES}
)

add_executable(test_mission
  src/test_mission.cpp
)

add_dependencies(test_mission
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(test_mission
  ${catkin_LIBRARIES}
)
