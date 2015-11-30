#include "dock_with_tape.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "line_pid_node");
  Line_follower line_follower(ros::this_node::getName()); // Error: error: invalid use of qualified-name 'ros::this_node::getName'
  ros::spin();
  return 0;
}
