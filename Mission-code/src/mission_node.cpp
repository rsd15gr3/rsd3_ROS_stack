#include <ros/ros.h>
#include <msgs/BoolStamped.h>
#include <msgs/IntStamped.h>
#include <std_msgs/Int32.h>
#include "../../defines/action_states.h"
#include "deque"

using namespace std;
using namespace ros;

void State_pick();

int loopRate = 10;
msgs::BoolStamped exampleMsg;

//
int currentAction = CTR_MANUAL;
bool manualControlActive = true;
bool verificationBox = false;
bool verificationGPS = false;
bool verificationLine = false;
bool verificationTipper = false;
deque<int> actionList;

//Subscribers
ros::Subscriber deque_subscriber;
ros::Subscriber manualcontrol_subscriber;
ros::Subscriber verificationBox_subscriber;
ros::Subscriber verificationGPS_subscriber;
ros::Subscriber verificationTipper_subscriber;
ros::Subscriber verificationLine_subscriber;

//Publishers
ros::Publisher actionstate_publisher;

void dequeCallback(const msgs::IntStamped::ConstPtr& msg)
{
    msgs::IntStamped _msg = *msg;
    actionList.push_front(_msg.data);
    State_pick();
}

void manualcontrolCallback(const msgs::BoolStamped::ConstPtr& msg)
{
    msgs::BoolStamped _msg = *msg;
    manualControlActive = _msg.data;
    State_pick();
}

void verificationBoxCallback(const msgs::BoolStamped::ConstPtr& msg)
{
    msgs::BoolStamped _msg = *msg;
    verificationBox = _msg.data;
    State_pick();
}

void verificationGPSCallback(const msgs::BoolStamped::ConstPtr& msg)
{
    msgs::BoolStamped _msg = *msg;
    verificationGPS = _msg.data;
    State_pick();
}

void verificationLineCallback(const msgs::BoolStamped::ConstPtr& msg)
{
    msgs::BoolStamped _msg = *msg;
    verificationLine = _msg.data;
    State_pick();
}

void verificationTipperCallback(const msgs::BoolStamped::ConstPtr& msg)
{
    msgs::BoolStamped _msg = *msg;
    verificationTipper = _msg.data;
    State_pick();
}

void exampleCallback(const msgs::BoolStamped::ConstPtr& msg)
{
	exampleMsg = *msg;
    cout << "Callback: Recived message" << endl;
}

int main(int argc, char **argv){

	ros::init(argc, argv, "mission_node");
	NodeHandle nodeHandler;
    //nodeHandler.param<int>("loopRate", loopRate, 10);

    //init subscribers
    deque_subscriber = nodeHandler.subscribe("mission/deque", 20, dequeCallback);
    manualcontrol_subscriber = nodeHandler.subscribe("perception/manualcontrol_verify", 1, manualcontrolCallback);
    ros::Subscriber example_subscriber = nodeHandler.subscribe("perception/example_verify_pub", 1, exampleCallback);
    verificationBox_subscriber = nodeHandler.subscribe("perception/box_verify", 1, verificationBoxCallback);
    verificationGPS_subscriber = nodeHandler.subscribe("perception/gps_verify", 1, verificationGPSCallback);
    verificationLine_subscriber = nodeHandler.subscribe("perception/line_verify", 1, verificationLineCallback);
    verificationTipper_subscriber = nodeHandler.subscribe("perception/tipper_verify", 1, verificationTipperCallback);

    //init publishers
    actionstate_publisher = nodeHandler.advertise<msgs::IntStamped>("mission/action_state",1);

    ros::spin();
    
	return 0;
}

void State_pick()
{
    if(manualControlActive)
    {
        currentAction = CTR_MANUAL;
    }
    else
    {
        if(actionList.empty())
        {
            currentAction = CTR_IDLE;
        }

        if(actionList.size() > 1)
        {
            switch(actionList[1])
            {
                case BOX_BRICK: //these go to the same logic
                case BOX_CHARGE:
                case BOX_DOOR:
                    if(verificationBox)
                    {
                        actionList.pop_front();
                    }
                break;

                case GPS_DOOR: //these go to the same logic
                case GPS_LINE:
                    if(verificationGPS)
                    {
                        actionList.pop_front();
                    }
                break;

                case LINE_GPS: //these go to the same logic
                case LINE_TIPPER:
                    if(verificationLine)
                    {
                        actionList.pop_front();
                    }

                case TIPPER_UP: //these go to the same logic
                case TIPPER_DOWN:
                    if(verificationTipper)
                    {
                        actionList.pop_front();
                    }
                break;

                default:
                throw("missing states");
                break;
            }
        }

        currentAction = actionList.front();

    }

    msgs::IntStamped message;
    message.header.stamp = ros::Time::now();
    message.data = currentAction;
    actionstate_publisher.publish(message);

}


