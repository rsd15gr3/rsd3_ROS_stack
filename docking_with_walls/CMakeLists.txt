cmake_minimum_required(VERSION 2.8.3)
project(docking_with_walls)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  msgs
  nav_msgs
  actionlib
  actionlib_msgs
  message_generation
)

# setup.py
catkin_python_setup()

# Search dirs for includes
include_directories(${catkin_INCLUDE_DIRS})

# Check launch files in /launch for errors
#roslaunch_add_file_check(launch)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES docking_with_walls.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs  std_msgs nav_msgs
)

catkin_package(
  #INCLUDE_DIRS
  LIBRARIES actionlib
  CATKIN_DEPENDS actionlib actionlib_msgs message_runtime rospy std_msgs
  #DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(relative_move_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(relative_move_server_node src/relative_move_server_node.cpp)
#catkin_install_python(PROGRAMS src/docking_with_walls_node.py
 # DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(relative_move_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
#target_link_libraries(docking_with_walls_node
#  ${catkin_LIBRARIES}
#)
