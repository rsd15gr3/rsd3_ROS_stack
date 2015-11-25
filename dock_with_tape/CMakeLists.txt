cmake_minimum_required(VERSION 2.8.3)
project(dock_with_tape)

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

add_action_files(DIRECTORY action FILES DockWithTape.action)
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)
catkin_package(DEPENDS message_runtime)

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
add_executable(dock_with_tape_node ${SRC_LIST})

add_dependencies(dock_with_tape_node dock_with_tape_gencpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(dock_with_tape_node
  ${catkin_LIBRARIES}
)
