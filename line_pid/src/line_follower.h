#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
#include "pid_controller.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <msgs/BoolStamped.h>
#include <line_detection/line.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <line_pid/FollowLineAction.h>
#include <relative_move_server/RelativeMoveAction.h>

class Line_follower
{
public:
  Line_follower(std::string name);
  std::string name_;
  // ros setup
  ros::NodeHandle nh;
  ros::Publisher pid_debug_pub, command_pub;
  ros::Subscriber error_sub, qr_tag_detect_sub, odometry_sub;
  ros::ServiceClient get_qr_client;
  // Line follower variables
  Pid_controller heading_controller;
  double angle_error, dist_error; // line errors
  double forward_speed, target_dist; // line follow params
  double ramp_speed, ramp_distance, stop_point_tolerance;
  ros::Timer timerPid;
  // stop at crossing variables
  const std::string camera_frame_id = "/camera_link";
  const std::string base_footprint_id = "/base_footprint";  
  double initial_distance_to_tag;
  bool line_follow_enabled, aligning_with_crossing;// state variables
  tf::Stamped<tf::Pose> odom_to_tag_transform;
  // action server
  actionlib::SimpleActionServer<line_pid::FollowLineAction> as_;
  line_pid::FollowLineResult result_;
  std::string stopping_qr_tag; // goal
  double stop_before_tag_dist;
  // functions
  void publishVelCommand(double forward_speed, double angular_speed);
  void pidCb(const ros::TimerEvent &);
  void lineCb(const line_detection::line::ConstPtr &linePtr);
  void odometryCb(const nav_msgs::Odometry &msg);
  void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered);
  double getMovingGoalTargetAngle();
  void goalCb();
  void preemtCb();
  // relative move
  actionlib::SimpleActionClient<relative_move_server::RelativeMoveAction> relative_move_ac_;
  relative_move_server::RelativeMoveGoal back_up_goal;
};

#endif // LINE_FOLLOWER_H
