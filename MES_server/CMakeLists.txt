cmake_minimum_required(VERSION 2.8.3)
project(mes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  msgs
  roscpp
  std_msgs
  mission
)
catkin_package(CATKIN_DEPENDS )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${mission_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(mes_node src/mes_node.cpp src/Node.cpp)
add_dependencies(mes_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(mes_node
   ${catkin_LIBRARIES}
 )

