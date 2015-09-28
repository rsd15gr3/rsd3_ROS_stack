#include "pid_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <msgs/row.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <vector>

using namespace std;
using namespace ros;

void wallCallback(const msgs::row::ConstPtr &wallPtr);
void wallEnableCallback(const msgs::IntStamped &enable);
double getMovingGoalTargetAngle();
void callbackPid(const ros::TimerEvent &);

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
bool wallFollowEnabled = false;

string wallsTopicName = "";
string automodeTopicName = "";
string pidDebugPubName = "";
string commandPubName = "";

Pid_controller heading_controller;

int main(int argc, char **argv)
{
    init(argc, argv, "wall_follow_control_node");
    NodeHandle n("~");
    n.param<int>("update_rate", updateRate, 20);
    n.param<double>("drive_kp", kp, 10.0);
    n.param<double>("drive_ki", ki, 3.35);
    n.param<double>("drive_kd", kd, 10.0);
    n.param<double>("drive_feed_forward", feedForward, 0.0);
    n.param<double>("drive_max_output", maxOutput, 0.40);
    n.param<double>("drive_max_i", maxI, 0.1);
    n.param<double>("target_dist", targetDist, 0.6);
    n.param<double>("forward_speed", feedFordSpeed, 0.4);
    n.param<string>("walls_sub", wallsTopicName, "/walls");
    n.param<string>("automode_sub", automodeTopicName, "/automode");
    n.param<string>("pid_debug_pub", pidDebugPubName, "/debug/pid_pub");
    n.param<string>("command_pub", commandPubName, "/fmCommand/cmd_vel");

    ros::Subscriber errorSub = n.subscribe(wallsTopicName, 1, wallCallback);
    ros::Subscriber enableSub = n.subscribe(automodeTopicName, 1, wallEnableCallback);

    commandPub = n.advertise<geometry_msgs::TwistStamped>(commandPubName, 1);
    pidDebugPub = n.advertise<msgs::FloatArrayStamped>(pidDebugPubName, 1);

    double updateInterval = 1.0 / updateRate;
    ros::Timer timerPid = n.createTimer(ros::Duration(updateInterval), callbackPid);

    ros::Rate r(updateRate);

    heading_controller.set_parameters(kp, ki, kd, feedForward, maxOutput, maxI, updateInterval);
    spin();
    return 0;
}

void callbackPid(const ros::TimerEvent &)
{
    if (wallFollowEnabled) {
        geometry_msgs::TwistStamped twistedStamped;
        twistedStamped.twist.linear.x = feedFordSpeed;
        twistedStamped.header.stamp = ros::Time::now();
        double movingGoalTargetAngle = getMovingGoalTargetAngle();
        twistedStamped.twist.angular.z = heading_controller.update(movingGoalTargetAngle);
        commandPub.publish(twistedStamped);

        // publish PID debug msgs
        msgs::FloatArrayStamped pidDebugMsg;
        pidDebugMsg.header.stamp = ros::Time::now();
        pidDebugMsg.data = heading_controller.getLatestUpdateValues();
        pidDebugPub.publish(pidDebugMsg);
    } else {
        heading_controller.reset();
    }
}

void wallCallback(const msgs::row::ConstPtr &wallPtr)
{
    angleError = wallPtr->error_angle;
    distError = wallPtr->error_distance;
}

double getMovingGoalTargetAngle()
{
    double closetMovingGoalAngle = atan2(targetDist, distError);
    double movingGoalTargetAngle = M_PI_2 - closetMovingGoalAngle - angleError;
    return movingGoalTargetAngle;
}

void wallEnableCallback(const msgs::IntStamped &enable)
{
    wallFollowEnabled = (enable.data == 1);
}
