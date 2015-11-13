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

using namespace std;
using namespace ros;
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG

void lineCb(const line_detection::line::ConstPtr &wallPtr);
double getMovingGoalTargetAngle();
void pidCb(const ros::TimerEvent &);
void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered);
void odometryCb(const geometry_msgs::PoseWithCovarianceStamped &msg);
Publisher commandPub;
Publisher pidDebugPub;
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

string lineTopicName = "";
string actionTopicName = "";
string pidDebugPubName = "";
string commandPubName = "";

string camera_frame_id = "/camera_link";
string base_link_id = "/base_link";
string stopping_qr_tag = "wc_3_conveyor";
tf::StampedTransform camera_to_base_link_tf;

geometry_msgs::Point initial_position;
geometry_msgs::Point current_position;
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
    n.param<string>("action_topic", actionTopicName, "/mission/action_state");   
    n.param<string>("pid_debug_pub", pidDebugPubName, "/debug/pid_pub");
    n.param<string>("command_pub", commandPubName, "/fmCommand/cmd_vel");
    ros::Subscriber errorSub = n.subscribe(lineTopicName, 1, lineCb);
    ros::Subscriber qr_tag_detect_sub = n.subscribe("/tag_found", 1, qrTagDetectCb);
    ros::Subscriber odometry_sub = n.subscribe("/fmProcessors/robot_pose_ekf/odom_combined", 1, odometryCb);
    // PID control setup
    commandPub = n.advertise<geometry_msgs::TwistStamped>(commandPubName, 1);    
    pidDebugPub = n.advertise<msgs::FloatArrayStamped>(pidDebugPubName, 1);
    double updateInterval = 1.0 / update_rate;
    ros::Timer timerPid = n.createTimer(ros::Duration(updateInterval), pidCb);
    line_follow_enabled = true;    
    heading_controller.set_parameters(kp, ki, kd, feed_forward, max_output, max_i, updateInterval);
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
  commandPub.publish(twistedStamped);
}

void pidCb(const ros::TimerEvent &)
{
    if (line_follow_enabled) {
        double movingGoalTargetAngle = getMovingGoalTargetAngle();
        publishVelCommand(forward_Speed, heading_controller.update(movingGoalTargetAngle));
        // publish PID debug msgs TODO: disable debug publish
        msgs::FloatArrayStamped pidDebugMsg;
        pidDebugMsg.header.stamp = ros::Time::now();
        pidDebugMsg.data = heading_controller.getLatestUpdateValues();
        pidDebugPub.publish(pidDebugMsg);
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

void odometryCb(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  current_position = msg.pose.pose.position;
  if(aligning_with_crossing) {
    double dx = fabs(current_position.x - initial_position.x);
    double dy = fabs(current_position.y - initial_position.y);
    double traveled_dist = hypot(dx,dy);
    ROS_DEBUG("Distance left to cross %f", distance_to_tag - traveled_dist);
    if(traveled_dist > distance_to_tag)
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
          listener.waitForTransform(base_link_id, camera_frame_id, ros::Time(0), ros::Duration(5.0) );
          listener.transformPose(base_link_id,ros::Time(0),pose,camera_frame_id,pose_in_base_link);
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
        }
        ROS_DEBUG("pos in camera: [%f, %f, %f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
        ROS_DEBUG("pos in base link: [%f, %f, %f]", pose_in_base_link.pose.position.x, pose_in_base_link.pose.position.y, pose_in_base_link.pose.position.z);
        // Maybe TODO: reset odometry to avoid overflow?
        initial_position = current_position;
        double dx = fabs(pose_in_base_link.pose.position.x - initial_position.x);
        double dy = fabs(pose_in_base_link.pose.position.y - initial_position.y);
        distance_to_tag = hypot(dx,dy);
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
