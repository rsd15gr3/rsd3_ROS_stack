#include "ros/ros.h"
#include <msgs/BoolStamped.h>
#include <sstream>

bool deadman;

void deadmanCb(const msgs::BoolStamped::ConstPtr& msg){
    deadman = msg->data;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "arduinoTalker");

    ros::NodeHandle n;

    ros::Publisher deadman_pub = n.advertise<msgs::BoolStamped>("/fmSafe/deadman", 1);

    std::string deadman_sub_topic;
    n.param("deadman_sub", deadman_sub_topic, std::string("/arduino/deadman"));
    ros::Subscriber deadman_sub = n.subscribe(deadman_sub_topic, 1, deadmanCb);

    ros::Rate loop_rate(20);

    while(n.ok()){
        msgs::BoolStamped msg_pub;
        msg_pub.header.stamp = ros::Time::now();
        msg_pub.data = deadman;

        deadman_pub.publish(msg_pub);

        ros::spinOnce();

        loop_rate.sleep();
    }


    return 0;
}

