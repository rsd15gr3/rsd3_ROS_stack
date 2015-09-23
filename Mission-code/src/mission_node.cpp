#include<ros/ros.h>
//#include<mission/mission_msg.h>

using namespace std;
using namespace ros;
int loopRate = 10;

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_node");
	NodeHandle n;
	n.param<int>("loopRate", loopRate, 10);
	//Subscriber sub = n.subscribe("")
	//ros::Subscriber sub = n.subscribe("perception/wall_error", 1, wallCallback);
	//Publisher pub = n.advertise<lego_dispenser_msgs::wall_error>("/fmCommand/cmd_vel",1);
	//ros::Rate r(this->loopRate);
	/*while (ros::ok())
	{
	// perform kalman fusion
	r.sleep();
	}*/

	ros::spin();
    
	return 0;
}

