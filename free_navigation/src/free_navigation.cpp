#include "free_navigation.h"
#include<boost/bind.hpp>
#include <tf/tf.h>
#include <mission/action_states.h>
Navigation::Navigation(std::string name)
    : name_(name), as_(nh_, name, false),move_base_ac_("move_base", true), static_frame_id("map"),
      action_line_follow("/action_line_follow", true)
{
  ROS_INFO("Starting navigation");
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
  }
  //ros::Subscriber sub = n.subscribe(action_topic, 1, &actionCb);
  while(!move_base_ac_.waitForServer(ros::Duration(5.0)) && ros::ok()){
      ROS_INFO("Waiting for the move_base action server to come up");
  }
  while(!action_line_follow.waitForServer(ros::Duration(5.0)) && ros::ok())
  {
      ROS_INFO("Waiting for the action_line_follow server to come up");
  }
  nh_.param<std::string>("stopping_tag", stopping_tag, "wc_3_entrance");
  line_goal.dist = stop_dist_before_tag;
  ROS_DEBUG("Starting action server");
  as_.start();
}

void Navigation::goalCb()
{
  goal_ = as_.acceptNewGoal()->behavior_type;
  approachGoal();
  ROS_INFO_NAMED(name_,"New behavior accepted");
}

void Navigation::preemtCb()
{
  as_.setPreempted();
  ROS_INFO_NAMED(name_,"Behavior preemted");
  // cancel all actions
  move_base_ac_.cancelAllGoals();
}

void Navigation::doneCbLine(const actionlib::SimpleClientGoalState& state,
            const line_pid::FollowLineResultConstPtr& result)
{
  ROS_INFO("Distance to goal: %f", result->distance_to_goal);
  as_.setSucceeded(result_); // first set succeeded in collecting doneCb
}

void Navigation::approachGoal()
{
      move_base_ac_.cancelAllGoals();
      move_base_msgs::MoveBaseGoal goal_msg;
      switch (goal_) {
      case CHARGE:
          ROS_INFO("GOING TO CHARGER: %i", BOX_CHARGE);
          goal_msg.target_pose.pose = charge_pose_;
          break;
      case BRICK:
          ROS_INFO("GOING TO collect bricks: %i", BRICK);
          goal_msg.target_pose.pose = load_bricks_pose_;
          break;
      case DELIVERY:
          ROS_INFO("GOING TO DELIVERY: %i", DELIVERY);
          goal_msg.target_pose.pose = delivery_pose_;
          break;
      case TRANSITION:
          ROS_INFO("GOING TO line to manipulators: %i", TRANSITION);
          goal_msg.target_pose.pose = line_to_manipulator_pose_;
          break;
      default:
          //ac_.cancelAllGoals();
          ROS_ERROR_NAMED(name_,"Unknown behavior type: %i",goal_);
          as_.setAborted(result_, "Unknown behavior type recieved");
          return; // not a navigation command so do not navigate
      }
      ROS_INFO("(%f, %f)",goal_msg.target_pose.pose.position.x, goal_msg.target_pose.pose.position.y);
      goal_msg.target_pose.header.frame_id = static_frame_id;
      goal_msg.target_pose.header.stamp = ros::Time::now();
      move_base_ac_.sendGoal(goal_msg, boost::bind(&Navigation::doneCb,this,_1, _2),
                   boost::bind(&Navigation::activeCb, this),
                   boost::bind(&Navigation::feedbackCb, this, _1) );
}


void Navigation::doneCb(const actionlib::SimpleClientGoalState& state,
                        const move_base_msgs::MoveBaseResultConstPtr& result)
{
  if( move_base_ac_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_ERROR_NAMED(name_, "Move base failed to approach target");
    as_.setAborted(result_,"Move base failed to approach target");
  }
  //ROS_INFO("Finished in state: %s", state.getText().c_str());
  switch (goal_) {
  case CHARGE:
      ROS_INFO("Docking in CHARGER: %i", BOX_CHARGE);
      line_goal.qr_tag = stopping_tag;
      action_line_follow.sendGoal(line_goal, boost::bind(&Navigation::doneCbLine,this,_1,_2));
      break;
  case BRICK:
      ROS_INFO("going in to collect bricks: %i", BRICK);
      as_.setSucceeded(result_); // first set succeeded in collecting doneCb
      break;
  case DELIVERY:
      ROS_INFO("Tipping of at DELIVERY: %i", DELIVERY);
      as_.setSucceeded(result_);
      break;
  case TRANSITION:
      ROS_INFO("At transition: %i", TRANSITION);
      as_.setSucceeded(result_);
      break;
  default:
      //ac_.cancelAllGoals();
      return; // not a navigation command so do not navigate
  }
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
