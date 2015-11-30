#include <iostream>
#include <queue>
#include <string>

#include "route.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <msgs/IntStamped.h>
#include <msgs/BoolStamped.h>
#include <msgs/FloatStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include "mission/action_states.h"
#include <test_server/testAction.h>
#include "free_navigation/NavigateFreelyAction.h"

#define BRICK_ORDER_1   1
#define BRICK_ORDER_2   2
#define BRICK_ORDER_3   3
#define BRICK_DELIVERY  4

//Publishers
ros::Publisher action_publisher;
ros::Publisher pose_publisher;

//Subscribers
ros::Subscriber mission_subscriber;
ros::Subscriber automode_subscriber;
ros::Subscriber charge_subscriber;
ros::Subscriber voltage_subscriber;

Route path;
std::queue<int> mission_queue;

bool navigation_area = true;
bool active_behavior = false;
bool automode = true;
bool should_charge = false;
bool error = false;
double voltage = 13;
double voltage_filled = 14;

std::string get_string(int value);

void doneCb(const actionlib::SimpleClientGoalState& state,
            const test_server::testResultConstPtr& result)
{
    active_behavior = false;
    path.pop();
}

void doneCbFrom(const actionlib::SimpleClientGoalState& state,
                const test_server::testResultConstPtr& result)
{
    active_behavior = false;
    path.pop();

    //update navigation pose
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "markerlocator";
    msg.pose.pose.orientation.w = 0.458046469719;
    msg.pose.pose.orientation.x = 0;
    msg.pose.pose.orientation.y = 0;
    msg.pose.pose.orientation.z = 0.888928248835;
    msg.pose.pose.position.x = 5.23196131368;
    msg.pose.pose.position.y = 0.838434089243;
    msg.pose.pose.position.z = 0.0;
    const double covariance = 0.1;
    msg.pose.covariance[0] = covariance;
    msg.pose.covariance[7] = covariance;
    msg.pose.covariance[35] = covariance;
    pose_publisher.publish(msg);
    /*for(int i=0; i<std::sqrt(msg.pose.covariance.size()); i++)
    {
      msg.pose.covariance.at(i+i*std::sqrt(msg.pose.covariance.size())) = 0.1;
    }*/
}

void doneCbNavigation(const actionlib::SimpleClientGoalState& state,
            const free_navigation::NavigateFreelyResultConstPtr& result)
{
    if(result->state != free_navigation::NavigateFreelyResult::SUCCESS)
    {
        error = false;
        ROS_ERROR_NAMED("mission", "Move base failed to approach target");
    }
    else
    {
        error = true;
    }
    active_behavior = false;
    path.pop();
}

void missionCallback(const msgs::IntStamped::ConstPtr& msg)
{
    mission_queue.push(msg->data);
}

void automodeCallback(const msgs::IntStamped::ConstPtr& msg)
{
    automode = msg->data;
}

void chargeCallback(const msgs::BoolStamped::ConstPtr& msg)
{
    should_charge = msg->data;
}

void voltageCallback(const msgs::FloatStamped::ConstPtr& msg)
{
    voltage = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
	ros::NodeHandle nodeHandler;

    int loopRate;
    nodeHandler.param<int>("loopRate", loopRate, 10);
    ros::Rate rate(loopRate);

	//init publishers
    action_publisher = nodeHandler.advertise<std_msgs::String>("mission/next_mission",1);
    pose_publisher = nodeHandler.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);

    mission_subscriber = nodeHandler.subscribe("ui/mes", 5, missionCallback);
    automode_subscriber = nodeHandler.subscribe("fmPlan/automode", 5, automodeCallback);
    charge_subscriber = nodeHandler.subscribe("battery_monitor/too_low_battery", 1, chargeCallback);
    voltage_subscriber = nodeHandler.subscribe("battery_monitor/battery_level", 5, voltageCallback);

    actionlib::SimpleActionClient<test_server::testAction> action_test("test_server", true);
    actionlib::SimpleActionClient<free_navigation::NavigateFreelyAction> action_navigation("free_navigator", true);
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
        if(automode && !error)
        {

            if( path.empty() )
            {
                if(mission_queue.empty())
                {
                    //std::cout << "Current state: " << path.getCurrentState() << std::endl;
                    if(path.getCurrentState() != CHARGE && path.getCurrentState() != CELL_1 && path.getCurrentState() != CELL_2 && path.getCurrentState() != CELL_3)
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
                //change plan if in need of charge
                if(path.getCurrentState() != CHARGE &&
                   path.getCurrentState() != CELL_1 &&
                   path.getCurrentState() != CELL_2 &&
                   path.getCurrentState() != CELL_3 &&
                   should_charge == true)
                {
                    path.goChargeInterupt();
                }
                //waits with orders until proberly charged
                if( !(path.getCurrentState() == CHARGE && voltage < voltage_filled) )
                {
                    if(navigation_area)
                    {
                        if(path.next() == TRANSITION)
                        {
                            navigation_area = false;
                        }

                        free_navigation::NavigateFreelyGoal goal;
                        goal.behavior_type = path.next();
                        //goal.order = path.next();
                        action_navigation.sendGoal(goal, &doneCbNavigation);
                        active_behavior = true;
                    }
                    else
                    {
                        if(path.next() == TRANSITION)
                        {
                            navigation_area = true;
                            test_server::testGoal goal;
                            goal.order = path.getCurrentState();
                            action_from_cell.sendGoal(goal, &doneCbFrom);
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
            }
            //--------------------------------------------------------


            //path.infoRoute();
            //send message for gui
            std_msgs::String gui_message;
            //gui_message.header.stamp = ros::Time::now();
            if(!path.empty())
            {
                gui_message.data = get_string(path.next());
            }
            else
            {
                if(path.getCurrentState() == CHARGE)
                {
                    gui_message.data = "Wait now(Charge)";
                }
                else
                {
                    gui_message.data = "Wait now(Cell)";
                }
            }
            action_publisher.publish(gui_message);
        }
        else
        {
            action_navigation.cancelAllGoals();
            action_from_cell.cancelAllGoals();
            action_to_cell.cancelAllGoals();
            active_behavior = false;
        }

		ros::spinOnce();
        rate.sleep();
    }

	return 0;
}


std::string get_string(int value)
{
    std::string ret;
    switch (value)
    {
        case CTR_IDLE:
            ret = "Wait now";
        break;

        case BRICK:
            ret = "Get some bricks";
        break;

        case TRANSITION:
            ret = "Go to transition area";
        break;

        case CHARGE:
            ret = "Go to charge station";
        break;

        case DELIVERY:
            ret = "Deliver the bricks";
        break;

        case CELL_1:
            ret = "Go to workcell 1";
        break;

        case CELL_2:
            ret = "Go to workcell 2";
        break;

        case CELL_3:
            ret = "Go to workcell 3";
        break;
    }
    return ret;
}
