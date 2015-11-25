#include "dock_with_tape.h"
#include <ros/ros.h>
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG

int main(int argc, char **argv)
{
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif
  ros::init(argc, argv, "line_pid_node");
  Line_follower line_follower(ros::this_node::getName()); // Error: error: invalid use of qualified-name 'ros::this_node::getName'
  ros::spin();
  return 0;
}
