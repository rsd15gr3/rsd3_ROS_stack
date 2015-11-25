cmake_minimum_required(VERSION 2.8.3)
project(zbar_decoder)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/modules)

aux_source_directory(src/ SRC_LIST)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  msgs
  message_generation
  geometry_msgs
)

find_package(ZBar REQUIRED)
find_package( OpenCV REQUIRED )

add_service_files(
  FILES
  decode_qr.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS} ${ZBAR_INCLUDE_DIRS})

add_executable(zbar_decoder_node ${SRC_LIST})

add_dependencies(zbar_decoder_node ${catkin_EXPORTED_TARGETS} zbar_decoder_gencpp)

target_link_libraries(zbar_decoder_node
  ${OpenCV_LIBS}
  ${ZBAR_LIBRARIES}
  ${catkin_LIBRARIES}
)
