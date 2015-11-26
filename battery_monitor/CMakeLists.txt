cmake_minimum_required(VERSION 2.8.3)
project(battery_monitor)

find_package(catkin REQUIRED COMPONENTS
  msgs
  roscpp
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
aux_source_directory(src/ SRC_LIST)
add_executable(battery_monitor_node
  ${SRC_LIST}
)

add_dependencies(battery_monitor_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(battery_monitor_node
  ${catkin_LIBRARIES}
)
