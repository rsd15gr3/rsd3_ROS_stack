#include "free_navigation.h"
#include<boost/bind.hpp>
#include <tf/tf.h>
#include <mission/action_states.h>
Navigation::Navigation(std::string name)
    : name_(name), as_(nh_, name, false),move_base_ac_("move_base", true), static_frame_id("obstacle_map"),
      docking_with_walls_ac_("/docking_with_walls_server", true), relative_move_ac_("/relative_move_action", true),
      collect_bricks_ac_("/collect_bricks_pos_node", true)

{
  ROS_INFO_NAMED(name_,"Starting navigation");
  // setup action server
  as_.registerGoalCallback(boost::bind(&Navigation::goalCb, this) );
  as_.registerPreemptCallback(boost::bind(&Navigation::preemtCb, this) );

  // setup move base behavior
  {
      vector<double> tmp_in_plane_pose;
      nh_.param<vector<double>>("line_to_manipulator", tmp_in_plane_pose, vector<double>{0,0,0});
      line_to_manipulator_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
      nh_.param<vector<double>>("delivery_pose", tmp_in_plane_pose, vector<double>{0,0,0});
      delivery_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
      nh_.param<vector<double>>("load_bricks", tmp_in_plane_pose, vector<double>{0,0,0});
      load_bricks_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
      nh_.param<vector<double>>("charge", tmp_in_plane_pose, vector<double>{0,0,0});
      charge_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
      nh_.param<vector<double>>("recovery_pose", tmp_in_plane_pose, vector<double>{0,0,0});
      recovery_ = Navigation::convertVecToPose(tmp_in_plane_pose);
  }
  //ros::Subscriber sub = n.subscribe(action_topic, 1, &actionCb);
  while(!move_base_ac_.waitForServer(ros::Duration(5.0)) && ros::ok()){
      ROS_INFO_NAMED(name_,"Waiting for the move_base action server to come up");
  }
  while(!docking_with_walls_ac_.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
      ROS_INFO_NAMED(name_,"Waiting for the dock with walls server to come up");
  }
  while(!relative_move_ac_.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
      ROS_INFO_NAMED(name_,"Waiting for the relative move server to come up");
  }
  while(!collect_bricks_ac_.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
      ROS_INFO_NAMED(name_,"Waiting for the collect bricks pos server to come up");
  }
  current_position = Navigation::free;
  ROS_DEBUG("Starting action server");
  nh_.param<double>("undock_relative_move", undock_relative_move, -1.0);
  as_.start();
}

void Navigation::goalCb()
{
  goal_ = as_.acceptNewGoal()->behavior_type;
  relative_move_server::RelativeMoveGoal goal;
  switch (current_position) {
  case Navigation::docked:
      goal = getRelativeMove(undock_relative_move,0,0);
      relative_move_ac_.sendGoal(goal, boost::bind(&Navigation::doneRelativeMoveCb, this, _1, _2));
    break;
  case Navigation::under_dispenser:
      goal = getRelativeMove(-0.4,0,0.0);
      relative_move_ac_.sendGoal(goal, boost::bind(&Navigation::doneRelativeMoveCb, this, _1, _2));
    break;
  case Navigation::at_transition:
      approachGoal();
    break;
  case Navigation::free:
      approachGoal();
    break;
  default:
    break;
  }
  ROS_INFO_NAMED(name_,"New behavior accepted");
  in_recovery_mode = false;
}

void Navigation::doneRelativeMoveCb(const actionlib::SimpleClientGoalState& state,
                        const relative_move_server::RelativeMoveResultConstPtr& result)
{
  /* relative move returns failed even when it reaches a reasonable pose??
  if(result->end_state != relative_move_server::RelativeMoveResult::GOAL_REACHED)
  {
    ROS_ERROR_NAMED(name_, "Relative move failed, and ended in the state: %i", result->end_state);
    result_.state = free_navigation::NavigateFreelyResult::FAILED;
    as_.setAborted(result_,"Relative move failed");
    return;
  }
  */
  relative_move_server::RelativeMoveGoal goal;
  switch (current_position) {
  case Navigation::docked:
    goal = getRelativeMove(0,0,M_PI_4);
    relative_move_ac_.sendGoal(goal, boost::bind(&Navigation::doneRelativeMoveCb, this, _1, _2));
    current_position = Navigation::free;
    break;
  case Navigation::under_dispenser:
    goal = getRelativeMove(0,0,M_PI_2 + M_PI_4);
    relative_move_ac_.sendGoal(goal, boost::bind(&Navigation::doneRelativeMoveCb, this, _1, _2));
    current_position = Navigation::free;
    break;
  case Navigation::at_transition:
  case Navigation::free:
    approachGoal();
    current_position = Navigation::free;
    break;
  default:
    break;
  }

}

void Navigation::preemtCb()
{
  as_.setPreempted();
  ROS_INFO_NAMED(name_,"Behavior preemted");
  // cancel all actions
  move_base_ac_.cancelAllGoals();
  relative_move_ac_.cancelAllGoals();
  docking_with_walls_ac_.cancelAllGoals();
  collect_bricks_ac_.cancelAllGoals();
}

void Navigation::doneCbWalls(const actionlib::SimpleClientGoalState& state,
            const docking_with_walls::docking_with_wallsResultConstPtr& result)
{
  current_position = Navigation::docked;
  as_.setSucceeded(result_); // first set succeeded in collecting doneCb
}

void Navigation::approachGoal()
{
      move_base_ac_.cancelAllGoals();
      move_base_msgs::MoveBaseGoal goal_msg;
      switch (goal_) {
      case CHARGE:
          ROS_INFO_NAMED(name_,"GOING TO CHARGER: %i", BOX_CHARGE);
          goal_msg.target_pose.pose = charge_pose_;
          break;
      case BRICK:
          ROS_INFO_NAMED(name_,"GOING TO collect bricks: %i", BRICK);
          goal_msg.target_pose.pose = load_bricks_pose_;
          break;
      case DELIVERY:
          ROS_INFO_NAMED(name_,"GOING TO DELIVERY: %i", DELIVERY);
          goal_msg.target_pose.pose = delivery_pose_;
          break;
      case TRANSITION:
          ROS_INFO_NAMED(name_,"GOING TO line to manipulators: %i", TRANSITION);
          goal_msg.target_pose.pose = line_to_manipulator_pose_;
          break;
      default:
          //ac_.cancelAllGoals();
          ROS_ERROR_NAMED(name_,"Unknown behavior type: %i",goal_);
          result_.state = free_navigation::NavigateFreelyResult::ERROR;
          as_.setAborted(result_, "Unknown behavior type recieved");
          return; // not a navigation command so do not navigate
      }
      sendMoveBaseGoal(goal_msg);
}

void Navigation::sendMoveBaseGoal(move_base_msgs::MoveBaseGoal& goal_msg)
{
  //ROS_INFO_NAMED(name_,"(%f, %f)",goal_msg.target_pose.pose.position.x, goal_msg.target_pose.pose.position.y);
  goal_msg.target_pose.header.frame_id = static_frame_id;
  goal_msg.target_pose.header.stamp = ros::Time::now();
  move_base_ac_.sendGoal(goal_msg, boost::bind(&Navigation::doneCb,this,_1, _2),
               boost::bind(&Navigation::activeCb, this),
               boost::bind(&Navigation::feedbackCb, this, _1) );
}

void Navigation::doneCollectBricksCv(const actionlib::SimpleClientGoalState& state, const collect_bricks_pos::collect_bricks_posResultConstPtr& result)
{
  current_position = Navigation::under_dispenser;
  ros::Duration(5.0).sleep();
  as_.setSucceeded(result_);
}

void Navigation::doneCb(const actionlib::SimpleClientGoalState& state,
                        const move_base_msgs::MoveBaseResultConstPtr& result)
{
  if( move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_WARN_NAMED(name_, "Move base failed to approach target");
    move_base_ac_.cancelAllGoals();
    move_base_msgs::MoveBaseGoal goal_msg;
    ROS_INFO_NAMED(name_,"GOING TO recovery pose: %i", BOX_CHARGE);
    goal_msg.target_pose.pose = recovery_;
    sendMoveBaseGoal(goal_msg);
    in_recovery_mode = true;
    //result_.state = free_navigation::NavigateFreelyResult::FAILED;
    //as_.setAborted(result_,"Move base failed to approach target");
    return;
  }
  // if at staging area then approach previous goal again
  if(in_recovery_mode)
  {
    in_recovery_mode = false;
    approachGoal();
  }
  else
  {
    //ROS_INFO_NAMED(name_,"Finished in state: %s", state.getText().c_str());
    switch (goal_) {
    case CHARGE:
        ROS_INFO_NAMED(name_,"Docking in CHARGER: %i", BOX_CHARGE);
        {
          docking_with_walls::docking_with_wallsGoal dock_goal;
          docking_with_walls_ac_.cancelAllGoals();
          docking_with_walls_ac_.sendGoal(dock_goal, boost::bind(&Navigation::doneCbWalls,this,_1,_2));
        }
        break;
    case BRICK:
        ROS_INFO_NAMED(name_,"going in to collect bricks: %i", BRICK);
        {
          collect_bricks_pos::collect_bricks_posGoal collect_bricks_goal;
          collect_bricks_ac_.cancelAllGoals();
          collect_bricks_ac_.sendGoal(collect_bricks_goal, boost::bind(&Navigation::doneCollectBricksCv,this,_1,_2));
        }
        break;
    case DELIVERY:
        ROS_INFO_NAMED(name_,"Tipping of at DELIVERY: %i", DELIVERY);
        as_.setSucceeded(result_);
        current_position = Navigation::free;
        break;
    case TRANSITION:
        ROS_INFO_NAMED(name_,"At transition: %i", TRANSITION);
        as_.setSucceeded(result_);
        current_position = Navigation::at_transition;
        break;
    default:
        //ac_.cancelAllGoals();
        return; // not a navigation command so do not navigate
    }
  }
}
void Navigation::activeCb()
{
    //ROS_INFO_NAMED(name_,"Start executing Navigation command");
}

void Navigation::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
    //ROS_INFO_NAMED(name_,"Feedback update:");
    // Possible to Monitor position on feedback->base_position
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

relative_move_server::RelativeMoveGoal Navigation::getRelativeMove(double dx, double dy, double dth){
    relative_move_server::RelativeMoveGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = dx;
    goal.target_pose.pose.position.y = dy;
    goal.target_yaw = dth;
    return goal;
}
