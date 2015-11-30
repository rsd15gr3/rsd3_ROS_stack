#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "arduinoTalker");

  ros::NodeHandle n;

  ros::Publisher unload_pub = n.advertise<std_msgs::String>("unload", 1000);

  ros::Rate loop_rate(0.05);

  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream unload;
    unload << "unload";
    msg.data = unload.str();

    unload_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}

