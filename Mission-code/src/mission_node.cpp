#include <ros/ros.h>
#include <mutex>
#include <msgs/BoolStamped.h>
#include "../../defines/action_states.h"
#include <std_msgs/Int32.h>

using namespace std;
using namespace ros;

int loopRate = 10;
std::mutex exampleMsg_lock;
msgs::BoolStamped exampleMsg;

/*struct task
{
	state_type state;
	int target;
}*/


void exampleCallback(const msgs::BoolStamped::ConstPtr& msg)
{
	exampleMsg_lock.lock();
	exampleMsg = *msg;
	exampleMsg_lock.unlock();
	cout << "Callback: Recived message" << endl;

}

/*geometry_msgs::Twist State_pick()
{
	static bool switchTask = false;

	if(switchTask == true)
	{
	switchTask = false;
	//current_state = Queue_Tasks.front(); //get next task
	//Queue_Tasks.pop(); //remove task
	}
	int done = 0;
	//taskMessage msg;	
	geometry_msgs::Twist returnValue;

	switch(current_state.state)
	{
		case IDLE:
			exampleMsg_lock.lock();
			done = exampleMsg.done;
			exampleMsg_lock.unlock();
		break;
		case BRIDGE: 		
			//msg = bridge_function(current_state.target);
		break;
		case GPS:
			//msg = gps_function(current_state.target); 
		break;
		case LINE:
			//msg = line_function(current_state.target);
		case CHARGE:
			//msg = charge_function(current_state.target);
		break;
		case BACKTRACK:
			//Backtrack
		break;
		default:
			//YOU DIE!
		break;
	}

	switch(Queue_Tasks.front().state)
	{
		case IDLE:
			exampleMsg_lock.lock();
			switchTask = exampleMsg.verify;
			exampleMsg_lock.unlock();
		break;
		case BRIDGE: 		
			done = bridge_verification(Queue_Tasks.front().target);
		break;
		case GPS:
			done = gps_verification(Queue_Tasks.front().target); 
		break;
		case LINE:
			done = line_verification(Queue_Tasks.front().target);
		case CHARGE:
			done = charge_verification(Queue_Tasks.front().target);
		break;
		default:
			//YOU DIE!
		break;
	}
				
}*/


int main(int argc, char **argv){

	ros::init(argc, argv, "mission_node");
	NodeHandle nodeHandler;
	nodeHandler.param<int>("loopRate", loopRate, 10);

    ros::Subscriber example_subscriber = nodeHandler.subscribe("perception/example_verify_pub", 1, exampleCallback);
    ros::Publisher pub = nodeHandler.advertise<std_msgs::Int32>("mission/action_state",1);

    ros::Rate loop_rate(5); //freq (for testing)
	//ros::spinOnce();

    string verify;

	while(ros::ok())
	{
		exampleMsg_lock.lock();
        std_msgs::Int32 msg;
        if(exampleMsg.data)
        {
            verify = "true";
            msg.data = 0;
        }
        else
        {
            verify = "false";
            msg.data = 1;
        }
            cout << "Main: " << endl << "seq: " << exampleMsg.header.seq << endl << "Verify: " << verify << endl;
        pub.publish(msg);
		exampleMsg_lock.unlock();
		ros::spinOnce();

        loop_rate.sleep(); //no spamming (for testing)
	}
    
	return 0;
}

