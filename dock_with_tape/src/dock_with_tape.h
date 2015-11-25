#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H
#include "pid_controller.h"
#include <ros/ros.h>
#include <msgs/BoolStamped.h>
#include <line_detection/line.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib/server/simple_action_server.h>
#include <dock_with_tape/DockWithTapeAction.h>
class Line_follower
{
public:
  Line_follower(std::string name);
  std::string name_;
  // ros setup
  ros::NodeHandle nh;
  ros::Publisher pid_debug_pub, command_pub;
  ros::Subscriber error_sub, qr_tag_detect_sub, odometry_sub, laser_sub;
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
  geometry_msgs::Point current_position, tag_position; // to calculate vector from robot to tag
  double initial_distance_to_tag;
  bool line_follow_enabled, aligning_with_crossing;// state variables
  // action server
  actionlib::SimpleActionServer<dock_with_tape::DockWithTapeAction> as_;
  dock_with_tape::DockWithTapeResult result_;
  std::string stopping_qr_tag; // goal
  double stop_before_tag_dist;

  double distance_to_dock; // Distance to the docking given by the lidar
  // functions
  void publishVelCommand(double forward_speed, double angular_speed);
  void pidCb(const ros::TimerEvent &);
  void lineCb(const line_detection::line::ConstPtr &linePtr);
  void odometryCb(const nav_msgs::Odometry &msg);
  void laserCb(const sensor_msgs::LaserScan &laser);
  void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered);
  double getMovingGoalTargetAngle();
  void goalCb();
  void preemtCb();
};

#endif // LINE_FOLLOWER_H
