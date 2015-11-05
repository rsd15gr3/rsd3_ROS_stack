cmake_minimum_required(VERSION 2.8.3)
project(store_odom_poses)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(store_odom_poses_node src/store_odom_poses_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(store_odom_poses_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(store_odom_poses_node
  ${catkin_LIBRARIES}
)
