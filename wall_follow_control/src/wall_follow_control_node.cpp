#include "pid_controller.h"

#include<ros/ros.h>
#include<geometry_msgs/TwistStamped.h>
#include<msgs/row.h>
#include<msgs/BoolStamped.h>
#include<cmath>

using namespace std;
using namespace ros;

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
bool wallFollowEnabled = false;

void wallCallback(const msgs::row::ConstPtr& wallPtr);
void wallEnableCallback(const msgs::BoolStamped& enable);
double getMovingGoalTargetAngle();
string wallsTopicName = "";
string automodeTopicName = "";
int main(int argc, char **argv){
    init(argc, argv, "wall_follow_control_node");
    NodeHandle n("~");
    n.param<int>("update_rate", updateRate, 20);
    n.param<double>("drive_kp", kp, 10.0);
    n.param<double>("drive_ki", ki, 3.35);
    n.param<double>("drive_kd", kd, 10.0);
    n.param<double>("drive_feed_forward", feedForward, 0.0);
    n.param<double>("drive_max_output", maxOutput, 0.40);
    n.param<double>("target_dist",targetDist, 0.6);
    n.param<string>("walls_sub", wallsTopicName, "/walls");
    n.param<string>("automode_sub", automodeTopicName, "/walls");

    ros::Subscriber error_sub = n.subscribe(wallsTopicName, 1, wallCallback);
    ros::Subscriber enable_sub = n.subscribe(automodeTopicName,1,wallEnableCallback);
    Publisher pub = n.advertise<geometry_msgs::TwistStamped>("/fmCommand/cmd_vel",1);

    ros::Rate r(updateRate);
    Pid_controller heading_controller(updateRate, kp, ki, kd, feedForward, maxOutput);
    ROS_INFO("Starting wall following");
    while (n.ok())
    {
        if(wallFollowEnabled)
        {
            geometry_msgs::TwistStamped twistedStamped;
            // twistedStamped.twist.angular.z = 0.1; // yaw speed forward speed
            // twistedStamped.twist.linear.x = 0.1;
            // twistedStamped.header.stamp = ros::Time::now();
            twistedStamped.twist.linear.x = maxOutput;
            twistedStamped.header.stamp = ros::Time::now();
            double movingGoalTargetAngle = getMovingGoalTargetAngle();
            twistedStamped.twist.angular.z = heading_controller.update(movingGoalTargetAngle);
            ROS_INFO("twist angular z %f",twistedStamped.twist.angular.z );
            pub.publish(twistedStamped);
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
    return M_PI_2 - closetMovingGoalAngle - angleError;
}

void wallEnableCallback(const msgs::BoolStamped& enable)
{
    ROS_INFO("AUTO RECIEVED");
    wallFollowEnabled = enable.data;
}
