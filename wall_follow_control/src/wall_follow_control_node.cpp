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

void wallCallback(const msgs::row::ConstPtr& wallPtr);
void wallEnableCallback(const msgs::IntStamped& enable);
double getMovingGoalTargetAngle();

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
string pidDebugPub = "";
string commandPub = "";
int main(int argc, char **argv){
    init(argc, argv, "wall_follow_control_node");
    NodeHandle n("~");
    n.param<int>("update_rate", updateRate, 20);
    n.param<double>("drive_kp", kp, 10.0);
    n.param<double>("drive_ki", ki, 3.35);
    n.param<double>("drive_kd", kd, 10.0);
    n.param<double>("drive_feed_forward", feedForward, 0.0);
    n.param<double>("drive_max_output", maxOutput, 0.40);
    n.param<double>("drive_max_i", maxI, 0.1);
    n.param<double>("target_dist",targetDist, 0.6);
    n.param<double>("forward_speed", feedFordSpeed, 0.4);
    n.param<string>("walls_sub", wallsTopicName, "/walls");
    n.param<string>("automode_sub", automodeTopicName, "/automode");
    n.param<string>("pid_debug_pub", pidDebugPub, "/debug/pid_pub");
    n.param<string>("command_pub", commandPub, "/fmCommand/cmd_vel");

    ros::Subscriber error_sub = n.subscribe(wallsTopicName, 1, wallCallback);
    ros::Subscriber enable_sub = n.subscribe(automodeTopicName,1,wallEnableCallback);

    Publisher pub = n.advertise<geometry_msgs::TwistStamped>(commandPub,1);
    Publisher pid_debug_pub = n.advertise<msgs::FloatArrayStamped>(pidDebugPub,1);

    ros::Rate r(updateRate);

    Pid_controller heading_controller(kp, ki, kd, feedForward, maxOutput, maxI);
    while (n.ok())
    {
        if(wallFollowEnabled)
        {
            geometry_msgs::TwistStamped twistedStamped;
            // twistedStamped.twist.angular.z = 0.1; // yaw speed forward speed
            // twistedStamped.twist.linear.x = 0.1;
            // twistedStamped.header.stamp = ros::Time::now();
            twistedStamped.twist.linear.x = feedFordSpeed;
            twistedStamped.header.stamp = ros::Time::now();
            double movingGoalTargetAngle = getMovingGoalTargetAngle();
            Time Tnow = Time::now();
            double T = Tnow.toNSec() /1000000000.0;
            twistedStamped.twist.angular.z = heading_controller.update(movingGoalTargetAngle, T);
            pub.publish(twistedStamped);
            // publish PID debug msgs
            msgs::FloatArrayStamped pid_debug_msg;
            pid_debug_msg.header.stamp = ros::Time::now();
            pid_debug_msg.data = heading_controller.getLatestUpdateValues();
            pid_debug_pub.publish(pid_debug_msg);
        }
        else {
            heading_controller.reset();
        }
        spinOnce();
        r.sleep();
    }
    return 0;
}

void wallCallback(const msgs::row::ConstPtr& wallPtr)
{
    angleError = wallPtr->error_angle;
    distError = wallPtr->error_distance;
}

double getMovingGoalTargetAngle()
{
    double closetMovingGoalAngle = atan2(targetDist, distError);
    double movingGoalTargetAngle = M_PI_2 - closetMovingGoalAngle - angleError;
    return -movingGoalTargetAngle;
}

void wallEnableCallback(const msgs::IntStamped& enable)
{
    wallFollowEnabled = (enable.data == 1);
}
