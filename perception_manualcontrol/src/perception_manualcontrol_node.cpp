#include<ros/ros.h>
#include<msgs/BoolStamped.h>
#include <std_msgs/String.h>

using namespace std;
using namespace ros;
int loopRate = 5; //publish freq
ros::Publisher pub;

void ui_str_control_Callback(const std_msgs::String::ConstPtr& msg)
{
    std_msgs::String _msg = *msg;
    msgs::BoolStamped pubmsg;
    if(_msg.data == "mr_mode_manual")
    {
            pubmsg.data = true;
            cout << "control set to manual" << endl;
    }
    else if(_msg.data == "mr_mode_auto")
    {
            pubmsg.data = false;
            cout << "control set to auto" << endl;
    }
    else
        return;

    pubmsg.header.stamp = ros::Time::now();
    pub.publish(pubmsg);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "perception_manualcontrol_node");
    NodeHandle n;
    pub = n.advertise<msgs::BoolStamped>("perception/manualcontrol_verify",1);
    ros::Subscriber sub = n.subscribe("ui_str_control", 1, ui_str_control_Callback);
    ros::spin();
    return 0;
}

