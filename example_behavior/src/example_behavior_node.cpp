#include<ros/ros.h>
#include<mission/mission_msg.h> //include the mission_msg
#include<geometry_msgs/Twist.h> //include Twist_msg

using namespace std;
using namespace ros;
int loopRate = 2; //publish freq

int main(int argc, char **argv){

	ros::init(argc, argv, "example_behavior_node");
	NodeHandle n;
	ros::Publisher pub = n.advertise<mission::mission_msg>("example_behavior_node/msg",1);
	ros::Rate loop_rate(1);

	geometry_msgs::Twist cmd;
	cmd.angular.x = 1;
	cmd.angular.y = 2;
	cmd.angular.z = 3;
	cmd.linear.x = 3;
	cmd.linear.y = 2;
	cmd.linear.z = 1;

	while (ros::ok()) //Code loop
	{
	mission::mission_msg msg;
	msg.header.stamp = ros::Time::now();
	msg.done = 1;
	msg.fail = 0;
	msg.verify = 1;
	msg.velocity_cmd = cmd;
	pub.publish(msg);

	loop_rate.sleep(); //sleep to match pub freq
	
	}

	ros::spin();
    
	return 0;
}

