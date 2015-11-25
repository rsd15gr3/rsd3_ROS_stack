#include "dock_with_tape.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <vector>
#include <std_msgs/String.h>
#include <zbar_decoder/decode_qr.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "dock_with_tape/DockWithTapeAction.h"
using std::string;

Line_follower::Line_follower(string name)
  : nh("~"), as_(nh, name, false), name_(name)
{
  ROS_DEBUG("Starting line follower");
  // setup line follower pid
  int update_rate;
  double kp, ki, kd, feed_forward, max_output, max_i;
  string line_topic_name, command_pub_name, pid_debug_pub_name, laser_topic_name;
  nh.param<int>("update_rate", update_rate, 20);
  nh.param<double>("drive_kp", kp, 10.0);
  nh.param<double>("drive_ki", ki, 3.35);
  nh.param<double>("drive_kd", kd, 10.0);
  nh.param<double>("drive_feed_forward", feed_forward, 0.0);
  nh.param<double>("drive_max_output", max_output, 0.40);
  nh.param<double>("drive_max_i", max_i, 0.1);
  nh.param<double>("forward_speed", forward_speed, 0.1);
  nh.param<double>("target_dist", target_dist, 0.6);
  double update_interval = 1.0 / update_rate;
  timerPid = nh.createTimer(ros::Duration(update_interval), &Line_follower::pidCb, this);
  heading_controller.set_parameters(kp, ki, kd, feed_forward, max_output, max_i, update_interval);
  nh.param<string>("line_sub", line_topic_name, "/line_detector/perception/line");
  error_sub = nh.subscribe(line_topic_name, 1, &Line_follower::lineCb, this);
  nh.param<string>("pid_debug_pub", pid_debug_pub_name, "/debug/pid_pub");
  pid_debug_pub = nh.advertise<msgs::FloatArrayStamped>(pid_debug_pub_name, 1);
  nh.param<string>("command_pub", command_pub_name, "/fmCommand/cmd_vel");
  command_pub = nh.advertise<geometry_msgs::TwistStamped>(command_pub_name, 1);
  nh.param<string>("laser_sub", laser_topic_name, "/fmSensors/scan");
  laser_sub = nh.subscribe(laser_topic_name, 1, &Line_follower::laserCb, this);
  heading_controller.reset();
  line_follow_enabled = false;
  aligning_with_crossing = false;
  ramp_speed = forward_speed;
  // Setup stopping when docked

  // setup action server
  as_.registerGoalCallback(boost::bind(&Line_follower::goalCb, this) );
  as_.registerPreemptCallback(boost::bind(&Line_follower::preemtCb, this) );
  as_.start();
}

void Line_follower::goalCb()
{ 
  ramp_speed = forward_speed;
  line_follow_enabled = true;
  dock_goal = as_.acceptNewGoal();
  ROS_DEBUG_NAMED(name_,"Goal recieved. Going to this from the wall: %f", dock_goal->dist);
}

void Line_follower::preemtCb()
{
  publishVelCommand(0,0);
  heading_controller.reset();
  line_follow_enabled = false;
  as_.setPreempted();
  ROS_DEBUG_NAMED(name_,"Line follower is preemted");
}

void Line_follower::publishVelCommand(double forward_speed, double angular_speed)
{
  geometry_msgs::TwistStamped twistedStamped;
  twistedStamped.twist.linear.x = forward_speed;
  twistedStamped.header.stamp = ros::Time::now();
  twistedStamped.twist.angular.z = angular_speed;
  command_pub.publish(twistedStamped);
}

void Line_follower::pidCb(const ros::TimerEvent &)
{
  if (line_follow_enabled) {
    double movingGoalTargetAngle = getMovingGoalTargetAngle();
    publishVelCommand(ramp_speed, heading_controller.update(movingGoalTargetAngle));
    // publish PID debug msgs TODO: disable debug publish
    msgs::FloatArrayStamped pidDebugMsg;
    pidDebugMsg.header.stamp = ros::Time::now();
    pidDebugMsg.data = heading_controller.getLatestUpdateValues();
    pid_debug_pub.publish(pidDebugMsg);
  }
}

void Line_follower::lineCb(const line_detection::line::ConstPtr &linePtr)
{
  angle_error = linePtr->angle;
  dist_error = linePtr->offset;
  // TODO: abort if error is too big
  // as_.setAborted(result_);
}

double Line_follower::getMovingGoalTargetAngle()
{
  double closetMovingGoalAngle = atan2(target_dist, dist_error);
  double movingGoalTargetAngle = M_PI_2 - closetMovingGoalAngle - angle_error;
  return movingGoalTargetAngle;
}