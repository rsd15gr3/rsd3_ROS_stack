cmake_minimum_required(VERSION 2.8.3)
project(zbar_decoder)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)
set(CMAKE_BUILD_TYPE Relese) #  RelWithDebInfo
#set(ROS_COMPILE_FLAGS "-DROSCONSOLE_MIN_SEVERITY=4 ${ROS_COMPILE_FLAGS}") # Doesn't work  ROSCONSOLE_SEVERITY_DEBUG
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
	sensor_msgs
  cv_bridge
  image_transport
	message_generation
  msgs
)
find_package (ZBar REQUIRED)
find_package( OpenCV REQUIRED )

generate_messages(DEPENDENCIES std_msgs msgs)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(zbar_decoder_node src/zbar_decoder_node)

add_dependencies(zbar_decoder_node ZBar)

target_link_libraries(zbar_decoder_node 
  ${OpenCV_LIBS} 
  ${ZBAR_LIBRARIES}
  ${catkin_LIBRARIES}
)

