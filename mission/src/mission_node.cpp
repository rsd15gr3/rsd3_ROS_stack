#include "iostream"
#include "vector"

#include "route.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/action_client.h>
#include <actionlib/client/terminal_state.h>

#include <msgs/IntStamped.h>
#include "mission/action_states.h"

//Publishers
ros::Publisher action_publisher;

//Subscribers
ros::Subscriber state_subscriber;

bool navigation_area = true;
bool active_behavior = false;

/*void doneCb(const actionlib::SimpleClientGoalState& state,
                   const FibonacciResultConstPtr& result)
{
    active_behavior = false;
    path.pop();
}*/


int main(int argc, char **argv)
{
    Route path;

    ros::init(argc, argv, "mission_node");
	ros::NodeHandle nodeHandler;

    int loopRate;
    nodeHandler.param<int>("loopRate", loopRate, 10);
    ros::Rate rate(loopRate);

	//init publishers
    action_publisher = nodeHandler.advertise<msgs::IntStamped>("mission/next_mission",1);

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
        //check mes order---------------------------------
        /*if(mes=getBricks)
        {
            queue order
            path.brickOrder(CELL);
        }

        if(mes=Delivery)
        {
            queue order
            path.brickDelivery();
        }*/
        //-------------------------------------------------

        //fill up the next order if current is done--------
        //mission must never fill with more than 1 in this system
        if( path.empty() )
        {
            /*
            if(mission_queue.empty())
            {
                if(path.getCurrentState() != CHARGE || path.getCurrentState() != CELL_1 || path.getCurrentState() != CELL_2 || path.getCurrentState() != CELL_3)
                {
                    path.goCharge();
                }
            }
            else
            {
                switch(mission_queue.next())
                {
                    case: brickOrder
                    path.brickOrder(CELL);

                    case: brickDelivery
                    path.brickDelivery();

                }
            }
            */
        }
        //--------------------------------------------------


        //order next behavior if none is active
        if(!path.empty() && active_behavior == false)
        {
            if(navigation_area)
            {
                if(path.next() == TRANSITION)
                {
                    navigation_area = false;
                }
                //learning_actionlib::FibonacciGoal goal;
                //goal.order = path.next();
                //action_navigation.sendGoal(goal, doneCb);
                //action_navigation.waitForResult(); //default 0, which should mean blocking
                active_behavior = true;
            }
            else
            {
                if(path.next() == TRANSITION)
                {
                    navigation_area = true;
                    //learning_actionlib::FibonacciGoal goal;
                    //goal.order = path.next();
                    //action_from_cell.sendGoal(goal, , doneCb);
                    //action_from_cell.waitForResult(); //default 0, which should mean blocking
                    active_behavior = true;
                }
                else
                {
                    //learning_actionlib::FibonacciGoal goal;
                    //goal.order = path.next();
                    //action_to_cell.sendGoal(goal, doneCb);
                    //action_to_cell.waitForResult(); //default 0, which should mean blocking
                    active_behavior = true;
                }
            }
        }
        //--------------------------------------------------------

        //send message for gui
        msgs::IntStamped gui_message;
        gui_message.header.stamp = ros::Time::now();
        gui_message.data = path.next();
        action_publisher.publish(gui_message);

		ros::spinOnce();
        rate.sleep();
	}

	return 0;
}
