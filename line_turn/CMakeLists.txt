cmake_minimum_required(VERSION 2.8.3)
project(line_turn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  roslaunch
  line_detection
  nav_msgs
)

catkin_package(CATKIN_DEPENDS)

# Check launch files in launch/ for errors
# roslaunch_add_file_check(launch)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mission_INCLUDE_DIRS}
)

add_executable(line_turn_node src/turn.cpp)

add_dependencies(line_turn_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(line_turn_node
  ${catkin_LIBRARIES}
)


# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()
