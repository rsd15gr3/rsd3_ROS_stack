#include <ros/ros.h>
#include <string>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <test_server/testAction.h>
#include "line_pid/FollowLineAction.h"
#include "relative_move_server/RelativeMoveAction.h"
#include "msgs/IntStamped.h"
#include "mission/action_states.h"

int state_counter = 0;
const int nr_of_states = 10;
bool active_action = false;

void doneCb(const actionlib::SimpleClientGoalState& state,
            const test_server::testResultConstPtr& result)
{
    active_action = false;
    state_counter++;
}

void doneCbLine(const actionlib::SimpleClientGoalState& state,
            const line_pid::FollowLineResultConstPtr& result)
{
    active_action = false;
    state_counter++;
}

void doneCbMove(const actionlib::SimpleClientGoalState& state,
            const relative_move_server::RelativeMoveResultConstPtr& result)
{
    active_action = false;
    state_counter++;
}

relative_move_server::RelativeMoveGoal getRelativeMove(double dx, double dy, double dth){
    relative_move_server::RelativeMoveGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = dx;
    goal.target_pose.pose.position.y = dy;
    goal.target_yaw = dth;

    return goal;
}

void arduinoAnswerCb(msgs::IntStampedConstPtr msg)
{
  active_action = false;
  state_counter++;
}

class testAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<test_server::testAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  test_server::testFeedback feedback_;
  test_server::testResult result_;
  actionlib::SimpleActionClient<line_pid::FollowLineAction> action_line_follow;
  actionlib::SimpleActionClient<relative_move_server::RelativeMoveAction> action_free_navigation;
  actionlib::SimpleActionClient<test_server::testAction> action_tipper;
  ros::Publisher tipper_command_pub;
  ros::Subscriber tipper_state_sub;
  void state_pick(int cell, bool active);

public:

  testAction(std::string name) :
    as_(nh_, name, boost::bind(&testAction::executeCB, this, _1), false),
    action_name_(name),
    action_line_follow("/action_line_follow", true),
    action_free_navigation("/relative_move_action", true),
    action_tipper("action_tipper", true)
  {
    as_.start();
    if(action_line_follow.waitForServer() )
    {
        ROS_INFO("succesfully connected");
    }

    if(action_free_navigation.waitForServer() )
    {
        ROS_INFO("succesfully connected");
    }

    if(action_tipper.waitForServer() )
    {
        ROS_INFO("succesfully connected");
    }
    tipper_command_pub = nh_.advertise<msgs::IntStamped>("/arduino_goal", 1);
    tipper_state_sub = nh_.subscribe<msgs::IntStamped>("/arduino_answer", 1, );
  }

  ~testAction(void)
  {
  }

  void executeCB(const test_server::testGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(10); //rate of the action servor
    bool success = true;

    // start executing the action
    while(state_counter < nr_of_states) //this will most likely be while !done in the behaviors
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok()) //Important check so the server can be stopped while acting
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        state_pick(goal->order, false); // cancel current action
        break;
      }

      if(!active_action)
      {
          state_pick(goal->order, true);
      }

      feedback_.progress = state_counter;
      as_.publishFeedback(feedback_); //option to give feedback on how far it has gone
      ros::spinOnce();
      r.sleep();
    }

    if(success)
    {
      state_counter = 0;
      //result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_); //result given to the done callback in the client
    }
  }


};

void testAction::state_pick(int cell, bool active)
{
    std::cout << "executeing action: " << state_counter << std::endl;
    switch (state_counter)
    {
        //first follow line until QR-code
        case 0:
            if(active)
            {
                line_pid::FollowLineGoal goal;
                goal.dist = 0.0;
                active_action = true;

                switch (cell)
                {
                    case CELL_1:
                    goal.qr_tag = "wc_1_entrance";
                    break;

                    case CELL_2:
                    goal.qr_tag = "wc_2_entrance";
                    break;

                    case CELL_3:
                    goal.qr_tag = "wc_3_entrance";
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }
                action_line_follow.sendGoal(goal, &doneCbLine);

            }
            else
            {
                action_line_follow.cancelAllGoals();
            }
        break;
        //turn 90 degress right
        case 1:
            if(active)
            {
                relative_move_server::RelativeMoveGoal goal = getRelativeMove(0,0,-1.57);
                action_free_navigation.sendGoal(goal, &doneCbMove);
                active_action = true;
            }
            else
            {
                action_free_navigation.cancelAllGoals();
            }
        break;
        //Go to dropoff place
        case 2:
            if(active)
            {
                line_pid::FollowLineGoal goal;
                goal.dist = 0.25;
                switch (cell)
                {
                    case CELL_1:
                    goal.qr_tag = "wc_1_conveyor";
                    break;

                    case CELL_2:
                    goal.qr_tag = "wc_2_conveyor";
                    break;

                    case CELL_3:
                    goal.qr_tag = "wc_3_conveyor";
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }

                action_line_follow.sendGoal(goal, &doneCbLine);
                active_action = true;
            }
            else
            {
                action_line_follow.cancelAllGoals();
            }
        break;
        //Drop the bricks - tip up
        case 3:
            if(active)
            {
                msgs::IntStamped goal;
                goal.data = 0;
                goal.header.stamp = ros::Time::now();
                tipper_command_pub.publish(goal);
                active_action = true;
            }
            else
            {
            }
        break;
        //Drop the bricks - tip down
        case 4:
            if(active)
            {
                msgs::IntStamped goal;
                goal.data = 2;
                goal.header.stamp = ros::Time::now();
                tipper_command_pub.publish(goal);
                active_action = true;
            }
            else
            {
            }
        break;

        //Back off from the belt
        case 5:
            if(active)
            {
                relative_move_server::RelativeMoveGoal goal = getRelativeMove(-0.3,0,0);
                action_free_navigation.sendGoal(goal, &doneCbMove);
                active_action = true;
            }
            else
            {
                action_free_navigation.cancelAllGoals();
            }
        break;
        //Turn depending of cell
        case 6:
            if(active)
            {
                relative_move_server::RelativeMoveGoal goal;
                switch (cell)
                {
                    case CELL_1:
                    goal = getRelativeMove(0,0,-0.8);
                    break;

                    case CELL_2:
                    goal = getRelativeMove(0,0,0.8);
                    break;

                    case CELL_3:
                    goal = getRelativeMove(0,0,-0.8);
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }
                action_free_navigation.sendGoal(goal, &doneCbMove);
                active_action = true;
            }
            else
            {
                action_free_navigation.cancelAllGoals();
            }
        break;
        //Go to the other line
        case 7:
            if(active)
            {
                relative_move_server::RelativeMoveGoal goal;
                switch (cell)
                {
                    case CELL_1:
                    goal = getRelativeMove(0.45,0,0);
                    break;

                    case CELL_2:
                    goal = getRelativeMove(0.45,0,0);
                    break;

                    case CELL_3:
                    goal = getRelativeMove(0.35,0,0);
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }
                action_free_navigation.sendGoal(goal, &doneCbMove);
                active_action = true;
            }
            else
            {
                action_free_navigation.cancelAllGoals();
            }
        break;            
        //Turn depending of cell
        case 8:
            if(active)
            {
                relative_move_server::RelativeMoveGoal goal;
                switch (cell)
                {
                    case CELL_1:
                    goal = getRelativeMove(0.1,0,0.77);
                    break;

                    case CELL_2:
                    goal = getRelativeMove(0.1,0,-0.77);
                    break;

                    case CELL_3:
                    goal = getRelativeMove(0.1,0,0.77);
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }
                action_free_navigation.sendGoal(goal, &doneCbMove);
                active_action = true;
            }
            else
            {
                action_free_navigation.cancelAllGoals();
            }
        break;
        //Turn depending of cell
        case 9:
            if(active)
            {
                relative_move_server::RelativeMoveGoal goal;
                switch (cell)
                {
                    case CELL_1:
                    goal = getRelativeMove(0.2,0,0.4);
                    break;

                    case CELL_2:
                    goal = getRelativeMove(0.2,0,-0.4);
                    break;

                    case CELL_3:
                    goal = getRelativeMove(0.2,0,0.4);
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }
                action_free_navigation.sendGoal(goal, &doneCbMove);
                active_action = true;
            }
            else
            {
                action_free_navigation.cancelAllGoals();
            }
        break;        //Go to pickup place
        case 10:
            if(active)
            {
                line_pid::FollowLineGoal goal;
                goal.dist = 0.0;
                switch (cell)
                {
                    case CELL_1:
                    goal.qr_tag = "wc_1_load";
                    break;

                    case CELL_2:
                    goal.qr_tag = "wc_2_load";
                    break;

                    case CELL_3:
                    goal.qr_tag = "wc_3_load";
                    break;

                    default:
                    ROS_ERROR("Invalid cell pick");
                    break;
                }
                action_line_follow.sendGoal(goal, &doneCbLine);
                active_action = true;
            }
            else
            {
                action_line_follow.cancelAllGoals();
            }
        break;


        default:
            ROS_ERROR("Some to cell state logic went wrong");
        break;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "to_cell_behavior");

    std::string server_name;
    ros::NodeHandle n("~");
    n.param<std::string>("node_name", server_name, "action_to_cell");

    std::cout << ros::this_node::getName() << std::endl;
    std::cout << server_name << std::endl;
    //testAction server(ros::this_node::getName());
    testAction server(server_name);

    ROS_INFO("server started");

    ros::spin();

    return 0;
}

