#include<ros/ros.h>
#include<mission/mission_msg.h>

using namespace std;
using namespace ros;
int loopRate = 10;

void exampleCallback(const mission::mission_msg::ConstPtr& msg)
{
	cout << msg->header.seq << endl;

}

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_node");
	NodeHandle nodeHandler;
	n.param<int>("loopRate", loopRate, 10);

	ros::Subscriber example_subscriber = nodeHandle.subscribe("example_behavior_node/msg", 1, exampleCallback);
	
	ros::spinOnce();

	while(ros::ok())
	{
	


	}
    
	return 0;
}

