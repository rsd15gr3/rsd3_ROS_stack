cmake_minimum_required(VERSION 2.8.3)
project(action_sub_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  msgs
  mission
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(action_sub_example_node src/action_sub_example_node.cpp)
add_dependencies(action_sub_example_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(action_sub_example_node ${catkin_LIBRARIES})
