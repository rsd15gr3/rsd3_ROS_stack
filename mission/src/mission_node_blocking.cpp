#include "iostream"
#include "vector"

#include "route.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>

#include <msgs/IntStamped.h>
#include "mission/action_states.h"


int loopRate;

//Publishers
ros::Publisher action_publisher;

//Subscribers
ros::Subscriber state_subscriber;

int currentState = CHARGE;
int navigation_area = true;

int main(int argc, char **argv)
{
    Route path;
    //path.brickDelivery();
    path.brickOrder(CELL_1);
    path.infoRoute();

    ros::init(argc, argv, "mission_node");
	ros::NodeHandle nodeHandler;
    nodeHandler.param<int>("loopRate", loopRate, 10);

	//init publishers
    action_publisher = nodeHandler.advertise<msgs::IntStamped>("mission/next_mission",1);
    ros::Rate rate(loopRate);

    /*
    actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> action_navigation("fibonacci", true);
    actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> action_to_cell("fibonacci", true);
    actionlib::SimpleActionClient<learning_actionlib::FibonacciAction> action_from_cell("fibonacci", true);


    action_navigation.waitForServer();
    action_to_cell.waitForServer();
    action_from_cell.waitForServer();

    learning_actionlib::FibonacciGoal goal;
    goal.order = 20;
    ac.sendGoal(goal);
    */

	while(ros::ok())
	{
        //check mes order

        /*if(mes=getBricks)
        {
            path.brickOrder(CELL);
        }

        if(mes=Delivery)
        {
            path.brickDelivery();
        }*/


        if(path.empty() && path.getCurrentState() != CHARGE)
        {
            path.goCharge();
        }

        if(!path.empty() && path.next() != CTR_IDLE)
        {
            if(navigation_area)
            {
                if(path.next() == TRANSITION)
                {
                    navigation_area = false;
                }
                //learning_actionlib::FibonacciGoal goal;
                //goal.order = path.next();
                //action_navigation.sendGoal(goal);
                //action_navigation.waitForResult(); //default 0, which should mean blocking
                path.pop();
            }
            else
            {
                if(path.next() == TRANSITION)
                {
                    navigation_area = true;
                    //learning_actionlib::FibonacciGoal goal;
                    //action_from_cell.sendGoal();
                    //action_from_cell.waitForResult(); //default 0, which should mean blocking
                    path.pop();
                }
                else
                {
                    //learning_actionlib::FibonacciGoal goal;
                    //goal.order = path.next();
                    //action_to_cell.sendGoal(goal);
                    //action_to_cell.waitForResult(); //default 0, which should mean blocking
                    path.pop();
                }
            }
        }

        msgs::IntStamped gui_message;
        gui_message.header.stamp = ros::Time::now();
        gui_message.data = path.next();
        action_publisher.publish(gui_message);

		ros::spinOnce();
        rate.sleep();
	}

	return 0;
}
