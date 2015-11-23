#include "line_follower.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <vector>
#include <std_msgs/String.h>
#include <zbar_decoder/decode_qr.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "line_pid/FollowLineAction.h"

using std::string;

Line_follower::Line_follower(string name)
  : nh("~"), as_(nh, name, false), name_(name)
{
  ROS_DEBUG("Starting line follower");
  // setup line follower pid
  int update_rate;
  double kp, ki, kd, feed_forward, max_output, max_i;
  string line_topic_name, command_pub_name, pid_debug_pub_name;
  nh.param<int>("update_rate", update_rate, 20);
  nh.param<double>("drive_kp", kp, 10.0);
  nh.param<double>("drive_ki", ki, 3.35);
  nh.param<double>("drive_kd", kd, 10.0);
  nh.param<double>("drive_feed_forward", feed_forward, 0.0);
  nh.param<double>("drive_max_output", max_output, 0.40);
  nh.param<double>("drive_max_i", max_i, 0.1);
  nh.param<double>("forward_speed", forward_speed, 0.4);
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
  heading_controller.reset();
  line_follow_enabled = false;
  aligning_with_crossing = false;
  ramp_speed = forward_speed;
  // Setup stopping at crossing
  string odom_sub, tag_found_sub;
  nh.param<double>("ramp_dist", ramp_distance, 0.1);
  nh.param<double>("stop_point_tolerance", stop_point_tolerance, 0.01);
  nh.param<string>("tag_found_sub", tag_found_sub, "/tag_found");
  qr_tag_detect_sub = nh.subscribe(tag_found_sub, 1, &Line_follower::qrTagDetectCb, this);
  nh.param<string>("odom_sub", odom_sub, "odometry/filtered/local");
  odometry_sub = nh.subscribe(odom_sub, 1, &Line_follower::odometryCb, this);
  get_qr_client = nh.serviceClient<zbar_decoder::decode_qr>("/get_qr_id");
  // setup action server
  as_.registerGoalCallback(boost::bind(&Line_follower::goalCb, this) );
  as_.registerPreemptCallback(boost::bind(&Line_follower::preemtCb, this) );
  as_.start();
}

void Line_follower::goalCb()
{ 
  aligning_with_crossing = false;
  ramp_speed = forward_speed;
  line_follow_enabled = true;
  line_pid::FollowLineGoalConstPtr line_goal = as_.acceptNewGoal();
  stop_before_tag_dist = line_goal->dist;
  stopping_qr_tag = line_goal->qr_tag;
  ROS_DEBUG_NAMED(name_,"Goal recieved. Going to stop at tag with value: %s", stopping_qr_tag.c_str());
}

void Line_follower::preemtCb()
{
  publishVelCommand(0,0);
  heading_controller.reset();
  line_follow_enabled = false;
  aligning_with_crossing = false;
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

void Line_follower::odometryCb(const nav_msgs::Odometry &msg)
{
  current_position = msg.pose.pose.position;
  if(aligning_with_crossing)
  {
    double dx = tag_position.x - current_position.x;
    double dy = tag_position.y - current_position.y;
    double dist_to_tag = cos(angle_error)*hypot(dx,dy);
    dist_to_tag -= stop_before_tag_dist;
    ROS_DEBUG("Distance to tag: %f", dist_to_tag);
    ROS_DEBUG("dx = %f",dx);
    ROS_DEBUG("dy = %f",dy);
    if(dist_to_tag > ramp_distance)
    {
      ramp_speed = forward_speed;
    }
    else if(dist_to_tag > stop_point_tolerance)
    {
      const double ramp_p = forward_speed/ramp_distance;
      ramp_speed = ramp_p * dist_to_tag;
    }
    else
    {
      publishVelCommand(0,0);
      heading_controller.reset();
      line_follow_enabled = false;
      result_.distance_to_goal = dist_to_tag;
      as_.setSucceeded(result_);
    }
  }
}

void Line_follower::qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered)
{
  if(qr_tag_entered.data)
  {
    ROS_DEBUG("Stopping to read tag");
    publishVelCommand(-0.5,0);
    ros::Duration(0.1).sleep();
    publishVelCommand(0,0);
    ros::Duration(0.5).sleep();
    zbar_decoder::decode_qr qr_request;
    qr_request.request.trash = "";
    geometry_msgs::PoseStamped trash;
    qr_request.request.trash2 = trash;
    if(get_qr_client.call(qr_request))
    {
      string tag = qr_request.response.value;
      if(tag == stopping_qr_tag)
      {
        ROS_DEBUG("Going to stop at tag: %s", tag.c_str());
        geometry_msgs::PoseStamped pose = qr_request.response.qr_tag_pose;
        geometry_msgs::PoseStamped pose_in_base_link;
        tf::TransformListener listener;
        try{
          listener.waitForTransform("odom", camera_frame_id, ros::Time(0), ros::Duration(5.0) );
          listener.transformPose("odom",ros::Time(0),pose,camera_frame_id,pose_in_base_link);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        ROS_DEBUG("pos in camera: [%f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        ROS_DEBUG("pos in base link: [%f, %f, %f]", pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y, pose_in_base_link.pose.position.z);
        tag_position.x = pose_in_base_link.pose.position.x;
        tag_position.y = pose_in_base_link.pose.position.y;
        aligning_with_crossing = true;
      }
      else
      {
        ROS_DEBUG("Continuing past tag: %s", tag.c_str());
      }
    }
    else
    {
      ROS_ERROR("qr decoder server returned false");
    }
  }
  else
  {
    // tag moved out of view
    //publishVelCommand(0,0);
    //line_follow_enabled = false;
  }
}

