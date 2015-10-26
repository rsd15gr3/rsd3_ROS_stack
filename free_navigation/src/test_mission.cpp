#include<ros/ros.h>
#include <mission/action_states.h>
#include<msgs/IntStamped.h>
#include<iostream>
int main(int argc, char** argv){
    ros::init(argc,argv,"test_navigator");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<msgs::IntStamped>("mission/action_state",1);
    ros::Rate r(5);
    while (ros::ok()) {
        msgs::IntStamped intMsg;
        std::cout << "Type a command " << std::endl;
        char input;
        std::cin >> input;
        if(input == 'q') {
          ROS_INFO("Test misssion node shutting down");
            break;
        }
        intMsg.data = (input - '0');

        pub.publish(intMsg);
        r.sleep();
    }
    ros::shutdown();
    return 0;
}
