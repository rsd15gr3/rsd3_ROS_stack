#include "iostream"
#include "Node.h"
#include "vector"
#include <ros/ros.h>
#include <msgs/IntStamped.h>
#include "mission/action_states.h"


int loopRate = 10;

//Publishers
ros::Publisher action_publisher;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mission_node");
	ros::NodeHandle nodeHandler;
    	nodeHandler.param<int>("loopRate", loopRate, 10);

	//init publishers
	action_publisher = nodeHandler.advertise<msgs::IntStamped>("mission/deque",1);

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

