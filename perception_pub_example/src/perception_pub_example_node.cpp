#include<ros/ros.h>
#include<msgs/BoolStamped.h>

using namespace std;
using namespace ros;
int loopRate = 5; //publish freq

int main(int argc, char **argv){

    ros::init(argc, argv, "example_perception_node");
    NodeHandle n;
    ros::Publisher pub = n.advertise<msgs::BoolStamped>("perception/example_verify_pub",1);
    ros::Rate loop_rate(loopRate);

    bool verify = false; /* Verify is set true when is likley that a "preception" is valid,
                        (eksample: The line is present, so it will be possible to follow it)*/

    while (ros::ok()) //Code loop
    {
    verify = !verify;

    msgs::BoolStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.data = verify;

    pub.publish(msg);

    loop_rate.sleep(); //sleep to match pub freq for testing

    }

    ros::spin();

    return 0;
}

