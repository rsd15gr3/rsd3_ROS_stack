#include "pid_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <vector>
#include <mission/action_states.h>
#include <line_detection/line.h>
#include <std_msgs/String.h>
#include <msgs/BoolStamped.h>
#include <zbar_decoder/decode_qr.h>

using namespace std;
using namespace ros;

void lineCb(const line_detection::line::ConstPtr &wallPtr);
void lineEnableCb(const msgs::IntStamped &enable);
double getMovingGoalTargetAngle();
void pidCb(const ros::TimerEvent &);
void actionStateCb(const msgs::IntStamped& action_state);
void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered);
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

Pid_controller heading_controller;

int main(int argc, char **argv)
{
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
    ros::Subscriber enableSub = n.subscribe(actionTopicName, 1, lineEnableCb);
    ros::Subscriber qr_tag_detect_sub = n.subscribe("/tag_found", 1, qrTagDetectCb);
    commandPub = n.advertise<geometry_msgs::TwistStamped>(commandPubName, 1);
    pidDebugPub = n.advertise<msgs::FloatArrayStamped>(pidDebugPubName, 1);
    n.subscribe(actionTopicName, 1, &actionStateCb);
    double updateInterval = 1.0 / update_rate;
    ros::Timer timerPid = n.createTimer(ros::Duration(updateInterval), pidCb);
    line_follow_enabled = true;
    get_qr_client = n.serviceClient<zbar_decoder::decode_qr>("/get_qr_id");
    heading_controller.set_parameters(kp, ki, kd, feed_forward, max_output, max_i, updateInterval);
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
      /*
        geometry_msgs::TwistStamped twistedStamped;
        twistedStamped.twist.linear.x = forward_Speed;
        twistedStamped.header.stamp = ros::Time::now();
        double movingGoalTargetAngle = getMovingGoalTargetAngle();
        twistedStamped.twist.angular.z = heading_controller.update(movingGoalTargetAngle);
        commandPub.publish(twistedStamped);
        */
        double movingGoalTargetAngle = getMovingGoalTargetAngle();
        publishVelCommand(forward_Speed, heading_controller.update(movingGoalTargetAngle));
        // publish PID debug msgs TODO: disable debug publish
        msgs::FloatArrayStamped pidDebugMsg;
        pidDebugMsg.header.stamp = ros::Time::now();
        pidDebugMsg.data = heading_controller.getLatestUpdateValues();
        pidDebugPub.publish(pidDebugMsg);
    } else {
      publishVelCommand(0,0);
      heading_controller.reset();
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

void lineEnableCb(const msgs::IntStamped &enable)
{
    //line_follow_enabled = (enable.data == 1);
}
void actionStateCb(const msgs::IntStamped& action_state)
{
    if(action_state.data == LINE_GPS || action_state.data == LINE_TIPPER)
    {
        line_follow_enabled = true;
        ROS_INFO("line activated");
    }
}

void qrTagDetectCb(const msgs::BoolStamped& qr_tag_entered)
{
  if(qr_tag_entered.data)
  {
    publishVelCommand(0,0);
    ros::Duration(1.0).sleep();
    zbar_decoder::decode_qr qr_request;
    qr_request.request.trash = "";
    string tag;
    if(get_qr_client.call(qr_request))
    {
      tag = qr_request.response.value;
    }
    else
    {
      ROS_ERROR("qr decoder server returned false");
    }
    ROS_INFO("tag value: %s", tag.c_str());
    // decode qr tag
  }
  else
  {
     publishVelCommand(0,0);
     line_follow_enabled = false;
  }
}
