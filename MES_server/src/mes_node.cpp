#include "iostream"
#include "Node.h"
#include "vector"
#include <ros/ros.h>
#include <msgs/IntStamped.h>
#include "mission/action_states.h"


int loopRate = 10;

int currentState = BOX_CHARGE;

//Publishers
ros::Publisher action_publisher;

//Subscribers
ros::Subscriber state_subscriber;

void stateCallback(const msgs::IntStamped::ConstPtr& msg)
{
    msgs::IntStamped _msg = *msg;
    currentState = _msg.data;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_node");
	ros::NodeHandle nodeHandler;
    	nodeHandler.param<int>("loopRate", loopRate, 10);

	//init publishers
	action_publisher = nodeHandler.advertise<msgs::IntStamped>("mission/deque",1);

	//init subscribers
    	state_subscriber = nodeHandler.subscribe("mission/action_state", 1, stateCallback);


	Nodes graph;
	graph.Construct();
	//graph.Load("graph.txt");
	graph.Save("graph.txt");

	//vector<unsigned int> route = graph.WidthSearch(1,2);
	//for(int i=0; i< route.size(); i++)
	//{
	//std::cout << "nr visited: " << route[i] << std::endl;
	//}

	while(ros::ok())
	{
	
		ros::spinOnce();
	}

	return 0;
}

