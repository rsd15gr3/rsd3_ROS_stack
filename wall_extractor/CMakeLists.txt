cmake_minimum_required(VERSION 2.8.11)
project(frobit_lego_transporter)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslaunch
  msgs
)

# Install from setup.py
catkin_python_setup()

# Search dirs for includes
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Check launch files in launch/ for errors
roslaunch_add_file_check(launch)
