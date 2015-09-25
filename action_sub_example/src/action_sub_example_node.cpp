#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "../../defines/action_states.h"

using namespace std;
using namespace ros;

bool action_active = false;

void stateCallback(const std_msgs::Int32::ConstPtr& msg)
{
    std_msgs::Int32 state = *msg;
    if(state.data == TEST) //Change TEST to the corrent define (define's can be found in action_states.h), all action has a #define
        action_active = true;
    else
        action_active = false;

    cout << "Callback: Recived message" <<endl;

}

int main(int argc, char **argv){
    ros::init(argc, argv, "action_sub_example_node");

    NodeHandle nodeHandler;

    ros::Subscriber example_subscriber = nodeHandler.subscribe("mission/action_state", 1, stateCallback);

    ros::Rate loop_rate(5); //freq (for sleeping)
    while(ros::ok())
    {
        if(action_active == true)
        {
            cout << "Do work" << endl; //Do actions (active)

        }
        else
        {
            cout << "Do nothing" << endl; //Dont do anything (State not active)
        }
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
