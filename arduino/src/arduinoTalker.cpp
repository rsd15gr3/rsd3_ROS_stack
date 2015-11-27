#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduinoTalker");

  ros::NodeHandle n;

  ros::Publisher unload_pub = n.advertise<std_msgs::String>("unload", 1000);

  ros::Rate loop_rate(0.05);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream unload;
    unload << "unload";
    msg.data = unload.str();

    ROS_INFO("%s", msg.data.c_str());

    unload_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
  }


  return 0;
}

