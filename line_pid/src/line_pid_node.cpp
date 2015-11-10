#include "pid_controller.h"
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <msgs/IntStamped.h>
#include <cmath>
#include <msgs/FloatArrayStamped.h>
#include <vector>
#include <mission/action_states.h>
#include <line_detection/line.h>
/*#include <actionlib/server/simple_action_server.h>
#include <line_pid/DockingAction.h>*/

using namespace std;
using namespace ros;

void lineCb(const line_detection::line::ConstPtr &wallPtr);
void lineEnableCb(const msgs::IntStamped &enable);
double getMovingGoalTargetAngle();
void pidCb(const ros::TimerEvent &);
void actionStateCb(const msgs::IntStamped& action_state);

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

string lineTopicName = "";
string actionTopicName = "";
string pidDebugPubName = "";
string commandPubName = "";

Pid_controller heading_controller;

/*typedef actionlib::SimpleActionServer<line_pid::DockingAction> Server;

void executeDocking(const line_pid::DockingGoalConstPtr &goal, Server* as_)
{
    // helper variables
    ros::Rate r(20);
    bool success = true;
    line_pid::DockingFeedback feedback_;
    line_pid::DockingResult result_;

    // push_back the seeds for the fibonacci sequence

    // publish info to the console for the user
    ROS_INFO("Executing, creating fibonacci sequence of order %i with seeds %i, %i", goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action

      // check that preempt has not been requested by the client
      if (as_->isPreemptRequested() || !ros::ok())
      {
        //ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_->setPreempted();
        success = false;
        break;
      }

      // publish the feedback
      as_->publishFeedback(feedback_);
      r.sleep();

    if(success)
    {
      result_.sequence = feedback_.sequence;
      as_->setSucceeded(result_);
    }
}
*/

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
    n.param<string>("line_sub", lineTopicName, "/line_detector/perception/line");
    n.param<string>("action_topic", actionTopicName, "/mission/action_state");   
    n.param<string>("pid_debug_pub", pidDebugPubName, "/debug/pid_pub");
    n.param<string>("command_pub", commandPubName, "/fmCommand/cmd_vel");

    ros::Subscriber errorSub = n.subscribe(lineTopicName, 1, lineCb);
    ros::Subscriber enableSub = n.subscribe(actionTopicName, 1, lineEnableCb);

    /*Server server(n, "charging_action_server", boost::bind(&executeDocking, _1, &server), false);
    server.start();*/

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
        twistedStamped.twist.linear.x = feedFordSpeed;
        twistedStamped.header.stamp = ros::Time::now();
        double movingGoalTargetAngle = getMovingGoalTargetAngle();
        twistedStamped.twist.angular.z = heading_controller.update(movingGoalTargetAngle);
        commandPub.publish(twistedStamped);

        // publish PID debug msgs TODO: disable debug publish
        msgs::FloatArrayStamped pidDebugMsg;
        pidDebugMsg.header.stamp = ros::Time::now();
        pidDebugMsg.data = heading_controller.getLatestUpdateValues();
        pidDebugPub.publish(pidDebugMsg);
    } else {
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

