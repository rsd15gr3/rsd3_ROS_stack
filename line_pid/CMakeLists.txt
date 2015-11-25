cmake_minimum_required(VERSION 2.8.3)
project(line_pid)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  msgs
  roslaunch
  line_detection
  mission
  tf
  nav_msgs
  actionlib_msgs
  actionlib
  zbar_decoder
)

add_action_files(DIRECTORY action FILES FollowLine.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
catkin_package(CATKIN_DEPENDS)

# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

# Check launch files in launch/ for errors
# roslaunch_add_file_check(launch)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
aux_source_directory(src/ SRC_LIST)
add_executable(line_pid_node ${SRC_LIST})

add_dependencies(line_pid_node line_pid_gencpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(line_pid_node
  ${catkin_LIBRARIES}
)
