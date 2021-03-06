cmake_minimum_required(VERSION 2.8.3)
project(line_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  msgs
  message_generation
  genmsg
)

find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV version ${OpenCV_VERSION}")

add_message_files(
  FILES
  line.msg
)

add_service_files(
  FILES
  cross.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Compiler flags
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11 -Wall -Wextra" HAS_FLAGS)
if(HAS_FLAGS)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

## Declare a C++ executable
add_executable(line_detection_node src/line_detection_node.cpp )
add_dependencies(line_detection_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(line_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
