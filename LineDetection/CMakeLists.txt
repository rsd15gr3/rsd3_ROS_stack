cmake_minimum_required(VERSION 2.8)
project( Color )
find_package( OpenCV REQUIRED )
add_executable(Color Color.cpp )
target_link_libraries(Color ${OpenCV_LIBS} )


