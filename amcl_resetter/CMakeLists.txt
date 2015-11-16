cmake_minimum_required(VERSION 2.8.3)
project(amcl_resetter)

# Find dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  nav_msgs
)

# Install from setup.py
catkin_python_setup()

