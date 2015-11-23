#include <ros/ros.h>
#include <mission/action_states.h>
#include<msgs/IntStamped.h>
#include <tf/tf.h>
#include "free_navigation.h"
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_INFO //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG

using std::vector;
using std::string;
using geometry_msgs::Pose;

void actionCb(const msgs::IntStamped& cmd)
{
    ROS_INFO("Action: %i recieved", cmd.data);
}

int main(int argc, char** argv){
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
#endif
    ros::init(argc,argv,"navigator");
    Navigation navigation(ros::this_node::getName());
    ros::spin();
    return 0;
}
