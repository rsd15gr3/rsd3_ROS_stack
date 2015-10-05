cmake_minimum_required(VERSION 2.8.3)
project(frobit_lego_transporter)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

# This is a catkin package
catkin_package()

# Check launch files in launch/ for errors
roslaunch_add_file_check(launch)
