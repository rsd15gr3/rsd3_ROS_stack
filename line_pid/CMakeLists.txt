cmake_minimum_required(VERSION 2.8.3)
project(line_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  msgs
  roslaunch
  line_detection
  mission
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_action_files(
  DIRECTORY action
  FILES Docking.action
)

generate_messages(
   DEPENDENCIES
   std_msgs  
   actionlib_msgs
)

catkin_package(CATKIN_DEPENDS actionlib_msgs)

# Check launch files in launch/ for errors
# roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${mission_INCLUDE_DIRS}
)

add_executable(line_pid_node src/pid_controller.cpp src/pid_controller.h src/line_pid_node.cpp)

add_dependencies(line_pid_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(line_pid_node
  ${catkin_LIBRARIES}
)


# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
