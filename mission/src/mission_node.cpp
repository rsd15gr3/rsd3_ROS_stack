#include <iostream>
#include <queue>

#include "route.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <msgs/IntStamped.h>
#include "mission/action_states.h"
#include <test_server/testAction.h>

#define BRICK_ORDER_1   1
#define BRICK_ORDER_2   2
#define BRICK_ORDER_3   3
#define BRICK_DELIVERY  4

//Publishers
ros::Publisher action_publisher;

//Subscribers
ros::Subscriber mission_subscriber;

Route path;
std::queue<int> mission_queue;

bool navigation_area = true;
bool active_behavior = false;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const test_server::testResultConstPtr& result)
{
    active_behavior = false;
    path.pop();
    ROS_INFO("got the cb");
}

void missionCallback(const msgs::IntStamped::ConstPtr& msg)
{
    mission_queue.push(msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
	ros::NodeHandle nodeHandler;

    //path.brickOrder(CELL_1);

    int loopRate;
    nodeHandler.param<int>("loopRate", loopRate, 10);
    ros::Rate rate(loopRate);

	//init publishers
    action_publisher = nodeHandler.advertise<msgs::IntStamped>("mission/next_mission",1);

    ros::Subscriber sub = nodeHandler.subscribe("UI/mes", 3, missionCallback);

    actionlib::SimpleActionClient<test_server::testAction> action_test("test_server", true);
    actionlib::SimpleActionClient<test_server::testAction> action_navigation("action_navigation", true);
    actionlib::SimpleActionClient<test_server::testAction> action_to_cell("action_to_cell", true);
    actionlib::SimpleActionClient<test_server::testAction> action_from_cell("action_from_cell", true);

    if(action_test.waitForServer(ros::Duration(1)) )
    {
        ROS_INFO("succesfully connected");
    }

    if(action_navigation.waitForServer() )
    {
        ROS_INFO("succesfully connected");
    }

    if(action_to_cell.waitForServer() )
    {
        ROS_INFO("succesfully connected");
    }

    if(action_from_cell.waitForServer() )
    {
        ROS_INFO("succesfully connected");
    }

    //path.brickOrder(CELL_1);
    //mission_queue.push(BRICK_ORDER_2);
    //mission_queue.push(BRICK_DELIVERY);

    test_server::testGoal goal;
    action_test.sendGoal(goal );
    //action_test.sendGoal(goal, &doneCb );
    action_test.waitForResult(ros::Duration(10)); //default 0, which should mean blocking

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
            if(mission_queue.empty())
            {
                if(path.getCurrentState() != CHARGE || path.getCurrentState() != CELL_1 || path.getCurrentState() != CELL_2 || path.getCurrentState() != CELL_3)
                {
                    path.goCharge();
                }
            }
            else
            {
                switch(mission_queue.front())
                {
                    case BRICK_ORDER_1:
                    path.brickOrder(CELL_1);
                    break;

                    case BRICK_ORDER_2:
                    path.brickOrder(CELL_2);
                    break;

                    case BRICK_ORDER_3:
                    path.brickOrder(CELL_3);
                    break;

                    case BRICK_DELIVERY:
                    path.brickDelivery();
                    break;

                    default:
                    break;
                }
                mission_queue.pop();
            }
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

                test_server::testGoal goal;
                goal.order = path.next();
                action_navigation.sendGoal(goal, &doneCb);
                active_behavior = true;
            }
            else
            {
                if(path.next() == TRANSITION)
                {
                    navigation_area = true;
                    test_server::testGoal goal;
                    goal.order = path.next();
                    action_from_cell.sendGoal(goal, &doneCb);
                    active_behavior = true;
                }
                else
                {
                    test_server::testGoal goal;
                    goal.order = path.next();
                    action_to_cell.sendGoal(goal, &doneCb);
                    active_behavior = true;
                }
            }
        }
        //--------------------------------------------------------


        path.infoRoute();
        //send message for gui
        msgs::IntStamped gui_message;
        gui_message.header.stamp = ros::Time::now();
        if(!path.empty())
        {
            gui_message.data = path.next();
        }
        else
        {
            gui_message.data = CTR_IDLE;
        }
        action_publisher.publish(gui_message);

		ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
