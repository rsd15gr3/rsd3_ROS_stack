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
  ros::Subscriber error_sub, laser_sub;
  ros::ServiceClient get_qr_client;
  // Line follower variables
  Pid_controller heading_controller;
  double angle_error, dist_error; // line errors
  double forward_speed; // line follow params
  double ramp_speed, ramp_distance, target_dist;
  ros::Timer timerPid;
  bool line_follow_enabled;// state variables
  // action server
  actionlib::SimpleActionServer<dock_with_tape::DockWithTapeAction> as_;
  dock_with_tape::DockWithTapeResult result_;
  dock_with_tape::DockWithTapeGoalConstPtr dock_goal;
  // functions
  void publishVelCommand(double forward_speed, double angular_speed);
  void pidCb(const ros::TimerEvent &);
  void lineCb(const line_detection::line::ConstPtr &linePtr);
  void laserCb(const sensor_msgs::LaserScan &laser);
  double getMovingGoalTargetAngle();
  void goalCb();
  void preemtCb();
  double distance_to_dock;
};

#endif // LINE_FOLLOWER_H
