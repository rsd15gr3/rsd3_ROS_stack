#include "pid_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <msgs/StringStamped.h>
#include <vector>
#include <mission/action_states.h>
#include <line_detection/line.h>

using namespace std;
using namespace ros;

void lineCb(const line_detection::line::ConstPtr &wallPtr);
void lineEnableCb(const msgs::IntStamped &enable);
double getMovingGoalTargetAngle();
void pidCb(const ros::TimerEvent &);
void actionStateCb(const msgs::IntStamped& action_state);
void turningLeft(const msgs::StringStamped &qr_detected);
void encoder_left_callback(const msgs::IntStamped &ticks);

Publisher commandPub;
Publisher pidDebugPub;

int updateRate = 10;
double kp = 0;
double ki = 0;
double kd = 0;
double feedForward = 0;
double maxOutput = 0;
double targetDist = 0;
double angleError = 0;
double distError = 0;
double bearingDist = 0;
double maxI = 0;
double feedFordSpeed = 0;
bool lineFollowEnabled = false;
bool turningRight = false;

int moveForwardTicks = 0;
int positionZeroTicks = 0;

string lineTopicName = "";
string actionTopicName = "";
string pidDebugPubName = "";
string commandPubName = "";

string turningTopicName = "";
string encoderTopicName = "";

Pid_controller heading_controller;

int main(int argc, char **argv)
{
    init(argc, argv, "line_pid_node");
    NodeHandle n("~");
    n.param<int>("update_rate", updateRate, 20);
    n.param<double>("drive_kp", kp, 10.0);
    n.param<double>("drive_ki", ki, 3.35); //3.35
    n.param<double>("drive_kd", kd, 10.0); //10.0
    n.param<double>("drive_feed_forward", feedForward, 0.0);
    n.param<double>("drive_max_output", maxOutput, 0.40);
    n.param<double>("drive_max_i", maxI, 0.1);
    n.param<double>("target_dist", targetDist, 0.6);
    n.param<double>("forward_speed", feedFordSpeed, 0.4);

    n.param<string>("qr_sub", turningTopicName, "/perception/qr_tag");
    n.param<string>("left_encoder_sub", encoderTopicName, "/fmInformation/enconder_left");

    n.param<string>("line_sub", lineTopicName, "/line_detector/perception/line");
    n.param<string>("action_topic", actionTopicName, "/mission/action_state");   
    n.param<string>("pid_debug_pub", pidDebugPubName, "/debug/pid_pub");
    n.param<string>("command_pub", commandPubName, "/fmCommand/cmd_vel");

    ros::Subscriber leftEncoder = n.subscribe(encoderTopicName, 1, encoder_left_callback);
    ros::Subscriber turning = n.subscribe(turningTopicName, 1, turningLeft);
    ros::Subscriber errorSub = n.subscribe(lineTopicName, 1, lineCb);
    ros::Subscriber enableSub = n.subscribe(actionTopicName, 1, lineEnableCb);

    commandPub = n.advertise<geometry_msgs::TwistStamped>(commandPubName, 1);
    pidDebugPub = n.advertise<msgs::FloatArrayStamped>(pidDebugPubName, 1);
    n.subscribe(actionTopicName, 1, &actionStateCb);
    double updateInterval = 1.0 / updateRate;
    ros::Timer timerPid = n.createTimer(ros::Duration(updateInterval), pidCb);

    ros::Rate r(updateRate);

    heading_controller.set_parameters(kp, ki, kd, feedForward, maxOutput, maxI, updateInterval);
    spin();
    return 0;
}

void pidCb(const ros::TimerEvent &)
{
    if (lineFollowEnabled) {
        geometry_msgs::TwistStamped twistedStamped;
        twistedStamped.twist.linear.x = 0; //feedFordSpeed;
        twistedStamped.header.stamp = ros::Time::now();
        double movingGoalTargetAngle = getMovingGoalTargetAngle();
        twistedStamped.twist.angular.z = 0;//heading_controller.update(movingGoalTargetAngle);
        commandPub.publish(twistedStamped);

        // publish PID debug msgs TODO: disable debug publish
        msgs::FloatArrayStamped pidDebugMsg;
        pidDebugMsg.header.stamp = ros::Time::now();
        pidDebugMsg.data = heading_controller.getLatestUpdateValues();
        pidDebugPub.publish(pidDebugMsg);
    }

    else if (turningRight){

        if (moveForwardTicks - positionZeroTicks < 340){
            geometry_msgs::TwistStamped twistedStamped;
            twistedStamped.twist.linear.x = feedFordSpeed * 0.5;
            twistedStamped.header.stamp = ros::Time::now();
            twistedStamped.twist.angular.z = 0;
            commandPub.publish(twistedStamped);
        }

        if (moveForwardTicks - positionZeroTicks > 340 && moveForwardTicks - positionZeroTicks < 840){
            geometry_msgs::TwistStamped twistedStamped;
            twistedStamped.twist.linear.x = 0;
            twistedStamped.header.stamp = ros::Time::now();
            twistedStamped.twist.angular.z = - 0.4;
            commandPub.publish(twistedStamped);
        }   
        else {
            turningRight= false;
            lineFollowEnabled= true;
        }
    }
    else  {
        heading_controller.reset();
    }
}

void lineCb(const line_detection::line::ConstPtr &linePtr)
{
    angleError = linePtr->angle;
    distError = linePtr->offset;
}

double getMovingGoalTargetAngle()
{
    double closetMovingGoalAngle = atan2(targetDist, distError);
    double movingGoalTargetAngle = M_PI_2 - closetMovingGoalAngle - angleError;
    return movingGoalTargetAngle;
}

void lineEnableCb(const msgs::IntStamped &enable)
{
    lineFollowEnabled = (enable.data == 1);
}
void actionStateCb(const msgs::IntStamped& action_state)
{
    if(action_state.data == LINE_GPS || action_state.data == LINE_TIPPER)
    {
        lineFollowEnabled = true;
        ROS_INFO("line activated");
    }
}

void encoder_left_callback(const msgs::IntStamped &ticks){
    moveForwardTicks = moveForwardTicks + 1;
}

void turningLeft(const msgs::StringStamped &qr_detected)
{
    if (qr_detected.data == "wc_3_entrance")
    lineFollowEnabled = false;
    turningRight = true;
    positionZeroTicks = moveForwardTicks;
}
