#include <free_navigation/free_navigation.h>
#include <boost/bind.hpp>
#include <tf/tf.h>
#include <mission/action_states.h>


Navigation::Navigation()
    : ac_("move_base", true), char_client_("relative_move_action", true)
{
    moving_ = false;
    rel_move_done_ = false;
    //char_client_("charge battery", true);
    while(!ac_.waitForServer(ros::Duration(5.0)) && ros::ok()){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    while(!char_client_.waitForServer(ros::Duration(5.0)) && ros::ok()){
        ROS_INFO("Waiting for the relative_move action server to come up");
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

void Navigation::doneRelativeMovCb(const actionlib::SimpleClientGoalState& state,
                                   const relative_move_server::RelativeMoveResultConstPtr& result)
{
  //ROS_INFO("Finished in state [%s]", end_state.toString().c_str());
  //ROS_INFO("Answer: %i", result->end_pose);
  //ros::shutdown();
  rel_move_done_ = true;
  moving_ = false;
}

void Navigation::activeRelativeMovCb()
{
  ROS_INFO("Goal just went active");
}

void Navigation::feedbackRelativeMovCb(const relative_move_server::RelativeMoveFeedbackConstPtr& feedback)
{
  ROS_INFO("Got Feedback ");//, feedback->current_goal);
}


void Navigation::actionStateCb(const msgs::IntStamped& action_state)
{
    ROS_INFO("Action: %i recieved", action_state.data);
    if(action_state.data != prev_action_state_)
    {
        double dx, dy, dTh;
        ac_.cancelAllGoals();
        switch (action_state.data) {
        //Cases for waypoint navigation
        case BOX_CHARGE:
            ROS_INFO("GOING TO CHARGER: %i", BOX_CHARGE);
            setTargetWayPoint(charge_initial_pose_);
            break;
        case BOX_BRICK:
            ROS_INFO("GOING TO collect bricks: %i", BOX_BRICK);
            setTargetWayPoint(load_bricks_pose_);
            break;
        case GPS_LINE:
            ROS_INFO("GOING TO line: %i", GPS_LINE);
            setTargetWayPoint(line_from_manipulator_pose_);
            break;
        case GPS_DOOR:
            ROS_INFO("GOING TO line to manipulators: %i", GPS_DOOR);
            setTargetWayPoint(line_to_manipulator_pose_);
            ROS_WARN("GPS door is depleted");
            break;
        //Cases for relative movement navigation
        case TURN90LEFT:
            ROS_INFO("Turning 90º left: %i", TURN90LEFT);
            dx = turn_90_left_pose_.position.x;
            dy = turn_90_left_pose_.position.y;
            dTh = turn_90_left_pose_.position.z;
            //tf::getYaw(charge_dock_pose_.orientation);
            setRelativeMove(dx, dy, dTh);
        case TURN90RIGHT:
            ROS_INFO("Turning 90º right: %i", TURN90RIGHT);
            dx = turn_90_right_pose_.position.x;
            dy = turn_90_right_pose_.position.y;
            dTh = turn_90_right_pose_.position.z;
            setRelativeMove(dx, dy, dTh);
        case TURN180:
            ROS_INFO("Turning 180º: %i", TURN180);
            dx = turn_180_pose_.position.x;
            dy = turn_180_pose_.position.y;
            dTh = turn_180_pose_.position.z;
            setRelativeMove(dx, dy, dTh);

        default:
            //ac_.cancelAllGoals();
            return; // not a navigation command so do not navigate
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

bool Navigation::setRelativeMove(double dx, double dy, double dth){
    relative_move_server::RelativeMoveGoal goal;
    goal.target_pose.header.frame_id = base_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = dx;
    goal.target_pose.pose.position.y = dy;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(dth);

    ROS_INFO("(%f, %f)",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);

    if (!sendGoal(goal))
        return false;

    if (!moving_)
        moving_ = true;

    return true;
}

bool Navigation::sendGoal(relative_move_server::RelativeMoveGoal& goal){
    goal.target_pose.header.stamp = ros::Time::now();
    rel_move_done_ = false;
    char_client_.sendGoal(goal, boost::bind(&Navigation::doneRelativeMovCb, this, _1, _2),
                         boost::bind(&Navigation::activeRelativeMovCb, this),
                         boost::bind(&Navigation::feedbackRelativeMovCb, this, _1));

    return true;
}

bool Navigation::setTargetWayPoint(const geometry_msgs::Pose goalPose){
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.pose = goalPose;
    ROS_INFO("(%f, %f)",goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    goal.target_pose.header.frame_id = base_frame_id_;
    goal.target_pose.header.stamp = ros::Time::now();        
    ac_.sendGoal(goal, boost::bind(&Navigation::doneCb,this,_1, _2),
                 boost::bind(&Navigation::activeCb, this),
                 boost::bind(&Navigation::feedbackCb, this, _1) );
}
