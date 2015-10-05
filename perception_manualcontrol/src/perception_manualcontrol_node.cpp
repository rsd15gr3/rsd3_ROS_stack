#include<ros/ros.h>
#include <msgs/BoolStamped.h>
#include <msgs/IntStamped.h>
#include <std_msgs/String.h>
#include <stdio.h>


using namespace std;
using namespace ros;

int loopRate = 5; //publish freq
ros::Publisher manual_control_verify_publisher;


void ui_str_control_Callback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String _msg = *msg;
    cout << _msg.data << endl;
    if(_msg.data == "mr_mode_manual")
    {
        msgs::BoolStamped pubmsg;
        pubmsg.data = true;
        pubmsg.header.stamp = ros::Time::now();
        manual_control_verify_publisher.publish(pubmsg);
    }
    else if(_msg.data == "mr_mode_auto")
    {
        msgs::BoolStamped pubmsg;
        pubmsg.data = false;
        pubmsg.header.stamp = ros::Time::now();
        manual_control_verify_publisher.publish(pubmsg);
    }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "perception_manualcontrol_node");
    NodeHandle n;
    manual_control_verify_publisher = n.advertise<msgs::BoolStamped>("perception/manualcontrol_verify",1);
    ros::Subscriber sub = n.subscribe("ui_str_control", 1, ui_str_control_Callback);
    ros::spin();
    return 0;
}

