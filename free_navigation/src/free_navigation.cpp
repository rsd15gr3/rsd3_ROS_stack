#include <free_navigation/free_navigation.h>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <mission/action_states.h>
Navigation::Navigation()
    : ac_("move_base", true), char_client_("charge_action", true)
{
    //char_client_("charge battery", true);
    while(!ac_.waitForServer(ros::Duration(5.0)) && ros::ok()){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}


void Navigation::doneCb(const actionlib::SimpleClientGoalState& state,
                        const move_base_msgs::MoveBaseResultConstPtr& result)
{
  //ROS_INFO("Finished in state: %s", state.getText().c_str());
}
void Navigation::activeCb()
{
    //ROS_INFO("Start executing Navigation command");
}

void Navigation::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    //ROS_INFO("Feedback update:");
    // Possible to Monitor position on feedback->base_position
}

void Navigation::actionStateCb(const msgs::IntStamped& action_state)
{
    ROS_INFO("Action: %i recieved", action_state.data);
    if(action_state.data != prev_action_state_)
    {
        ac_.cancelAllGoals();
        move_base_msgs::MoveBaseGoal goal;
        switch (action_state.data) {
        case BOX_CHARGE:
            ROS_INFO("GOING TO CHARGER: %i", BOX_CHARGE);
            goal.target_pose.pose = charge_pose_;
            break;
        case BOX_BRICK:
            ROS_INFO("GOING TO collect bricks: %i", BOX_BRICK);
            goal.target_pose.pose = load_bricks_pose_;
            break;
        case GPS_LINE:
            ROS_INFO("GOING TO line: %i", GPS_LINE);
            goal.target_pose.pose = line_from_manipulator_pose_;
            break;
        case GPS_DOOR:
            ROS_INFO("GOING TO line to manipulators: %i", GPS_DOOR);
            goal.target_pose.pose = line_to_manipulator_pose_;
            ROS_WARN("GPS door is depleted");
            break;
        case BOX_DOOR:
            ROS_WARN("Box door is depleted");
        default:
            //ac_.cancelAllGoals();
            return; // not a navigation command so do not navigate
        }

    /*
        Pose testPose;
        testPose.position.x=3.305;
        testPose.position.y=0.366;
        testPose.position.z=0.0;
        testPose.orientation.x=0.0;
        testPose.orientation.y=0;
        testPose.orientation.z=0.9723;
        testPose.orientation.w=-0.2319;
         move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose = testPose;
        */
        ROS_INFO("(%f, %f)",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
        goal.target_pose.header.frame_id = base_frame_id_;
        goal.target_pose.header.stamp = ros::Time::now();        
        ac_.sendGoal(goal, boost::bind(&Navigation::doneCb,this,_1, _2),
                     boost::bind(&Navigation::activeCb, this),
                     boost::bind(&Navigation::feedbackCb, this, _1) );

        if (action_state.data == BOX_CHARGE && ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            //call server for charging task
            ROS_INFO("hello");
            line_pid::DockingGoal goal; 
            goal.order = 20;
            char_client_.sendGoal(goal);
        }
    }
    prev_action_state_ = action_state.data;
}

Pose Navigation::convertVecToPose(const vector<double>& poses)
{
    geometry_msgs::Pose pose;
    if(poses.size() != 3) {
        ROS_ERROR("Waypoint with id %i is invalid, shuting down");
        ros::shutdown();
    }
    pose.position.x = poses[0];
    pose.position.y = poses[1];
    pose.position.z = 0;
    pose.orientation = tf::createQuaternionMsgFromYaw(poses[2]);
    return pose;
}
