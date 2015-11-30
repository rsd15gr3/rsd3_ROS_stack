#include <ros/ros.h>
#include <mission/action_states.h>
#include<msgs/IntStamped.h>
#include <tf/tf.h>
#include "free_navigation.h"

using std::vector;
using std::string;
using geometry_msgs::Pose;


int main(int argc, char** argv){
    ros::init(argc,argv,"navigator");
    Navigation navigation(ros::this_node::getName());
    ros::spin();
    return 0;
}
