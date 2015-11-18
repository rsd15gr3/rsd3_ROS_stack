#include "pid_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <vector>
#include <mission/action_states.h>
#include <line_detection/line.h>
#include <std_msgs/String.h>
#include <msgs/BoolStamped.h>
#include <zbar_decoder/decode_qr.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/LinearMath/Transform.h>

using namespace std;
using namespace ros;
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG

void lineCb(const line_detection::line::ConstPtr &wallPtr);
double getMovingGoalTargetAngle();
void pidCb(const ros::TimerEvent &);
void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered);
void odometryCb(const geometry_msgs::PoseWithCovarianceStamped &msg);
Publisher pid_debug_pub;
Publisher odom_reset_pub;
Publisher command_pub;
ros::ServiceClient get_qr_client;
int update_rate = 10;
double kp = 0;
double ki = 0;
double kd = 0;
double feed_forward = 0;
double max_output = 0;
double target_dist = 0;
double angle_error = 0;
double dist_error = 0;
double bearing_dist = 0;
double max_i = 0;
double forward_Speed = 0;
bool line_follow_enabled = false;
double ramp_speed;
string lineTopicName = "";
string pidDebugPubName = "";
string odom_sub = "";
string command_pub_name = "";
string odom_reset_topic = "";
string camera_frame_id = "/camera_link";
string base_footprint_id = "/base_footprint";
string stopping_qr_tag = "wc_3_conveyor";
tf::StampedTransform camera_to_base_link_tf;

geometry_msgs::Point initial_position;
geometry_msgs::Point current_position;
geometry_msgs::Point tag_position;
//tf::Stamped<tf::Pose> odom_to_tag_transform;

bool aligning_with_crossing;
double distance_to_tag;

Pid_controller heading_controller;

int main(int argc, char **argv)
{
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
#endif
    init(argc, argv, "line_pid_node");
    NodeHandle n("~");
    n.param<int>("update_rate", update_rate, 20);
    n.param<double>("drive_kp", kp, 10.0);
    n.param<double>("drive_ki", ki, 3.35); //3.35
    n.param<double>("drive_kd", kd, 10.0); //10.0
    n.param<double>("drive_feed_forward", feed_forward, 0.0);
    n.param<double>("drive_max_output", max_output, 0.40);
    n.param<double>("drive_max_i", max_i, 0.1);
    n.param<double>("target_dist", target_dist, 0.6);
    n.param<double>("forward_speed", forward_Speed, 0.4);
    n.param<string>("line_sub", lineTopicName, "/line_detector/perception/line");     
    n.param<string>("odom_sub", odom_sub, "odometry/filtered/local");
    n.param<string>("pid_debug_pub", pidDebugPubName, "/debug/pid_pub");
    n.param<string>("odom_reset_pub", odom_reset_topic, "/initialpose");
    n.param<string>("command_pub", command_pub_name, "/fmCommand/cmd_vel");
    ros::Subscriber error_sub = n.subscribe(lineTopicName, 1, lineCb);
    ros::Subscriber qr_tag_detect_sub = n.subscribe("/tag_found", 1, qrTagDetectCb);
    ros::Subscriber odometry_sub = n.subscribe(odom_sub, 1, odometryCb);
    // PID control setup
    pid_debug_pub = n.advertise<msgs::FloatArrayStamped>(pidDebugPubName, 1);
    odom_reset_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>(odom_reset_topic,1);
    command_pub = n.advertise<geometry_msgs::TwistStamped>(command_pub_name, 1);
    double update_interval = 1.0 / update_rate;
    ros::Timer timerPid = n.createTimer(ros::Duration(update_interval), pidCb);
    ramp_speed = forward_Speed;
    line_follow_enabled = true;    
    heading_controller.set_parameters(kp, ki, kd, feed_forward, max_output, max_i, update_interval);
    // Qr pose estimation setup
    aligning_with_crossing = false;
    get_qr_client = n.serviceClient<zbar_decoder::decode_qr>("/get_qr_id");
    // get camera to base_link transform
    spin();
    return 0;
}

void publishVelCommand(double forward_speed, double angular_speed)
{
  geometry_msgs::TwistStamped twistedStamped;
  twistedStamped.twist.linear.x = forward_speed;
  twistedStamped.header.stamp = ros::Time::now();
  twistedStamped.twist.angular.z = angular_speed;
  command_pub.publish(twistedStamped);
}

void pidCb(const ros::TimerEvent &)
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

void lineCb(const line_detection::line::ConstPtr &linePtr)
{
    angle_error = linePtr->angle;
    dist_error = linePtr->offset;
}

double getMovingGoalTargetAngle()
{
    double closetMovingGoalAngle = atan2(target_dist, dist_error);
    double movingGoalTargetAngle = M_PI_2 - closetMovingGoalAngle - angle_error;
    return movingGoalTargetAngle;
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void odometryCb(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  current_position = msg.pose.pose.position;
  if(aligning_with_crossing) {
/*
    tf::Vector3 robot_pose(current_position.x, current_position.y, current_position.z);
    tf::Vector3 robot_relative_to_tag = odom_to_tag_transform * robot_pose;
    ROS_DEBUG("robot x in tag frame = %f",robot_relative_to_tag.x());
    if(robot_relative_to_tag.x() > 0)
    {
      publishVelCommand(0,0);
      heading_controller.reset();
      line_follow_enabled = false;
    }
*/
    double dx = tag_position.x - current_position.x;
    double dy = tag_position.y - current_position.y;
    double dist_to_tag = hypot(dx,dy);
    float ramp_p = 10.0f*forward_Speed;
    ramp_speed = ramp_p * dist_to_tag;
    ROS_DEBUG("Distance to goal: %f", dist_to_tag);
    if(fabs(ramp_speed) > forward_Speed)
    {
      ramp_speed = sign(ramp_speed)*forward_Speed;
    }
    ROS_DEBUG("dx = %f",dx);
    if(dx < 0)
    {
      publishVelCommand(0,0);
      heading_controller.reset();
      line_follow_enabled = false;
    }

  }
}

void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered)
{
  if(qr_tag_entered.data)
  {
    ROS_DEBUG("Stopping to read tag");
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
          listener.waitForTransform(base_footprint_id, camera_frame_id, ros::Time(0), ros::Duration(5.0) );
          listener.transformPose(base_footprint_id,ros::Time(0),pose,camera_frame_id,pose_in_base_link);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        ROS_DEBUG("pos in camera: [%f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        ROS_DEBUG("pos in base link: [%f, %f, %f]", pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y, pose_in_base_link.pose.position.z);
        // Maybe TODO: reset odometry to avoid overflow?
        initial_position = current_position;
        geometry_msgs::PoseWithCovarianceStamped reset_pose;
        reset_pose.header.stamp = ros::Time::now();
        reset_pose.header.frame_id = pose.header.frame_id;
        reset_pose.pose.pose = pose.pose; // covariance at zero
        odom_reset_pub.publish(reset_pose);
        //tf::poseStampedMsgToTF(pose_in_base_link, odom_to_tag_transform);
        tag_position.x = current_position.x + pose_in_base_link.pose.position.x;
        tag_position.y = current_position.y + pose_in_base_link.pose.position.y;
        // distance_to_tag -= 0.2; // hot fix to stop at cross
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
