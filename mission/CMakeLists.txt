cmake_minimum_required(VERSION 2.8.3)
project(mission)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a C++ executable
add_executable(mission_node src/mission_node.cpp )
add_dependencies(mission_node ${catkin_EXPORTED_TARGETS})


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(mission_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(mission_node
   ${catkin_LIBRARIES}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
