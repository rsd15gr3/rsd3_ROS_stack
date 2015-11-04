#include <ros/ros.h>
#include <vector>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <math.h>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <line_detection/line.h>

void lineCb(const line_detection::line &msg);
void odometryCb(const nav_msgs::Odometry &msg);

line_detection::line line_info;
//nav_msgs::Odometry odometry_info;
tf::Quaternion odometry_info;

std::string lineTopicName;
std::string odometryTopicName;
std::string commandPubName;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_turn_node");
    ros::NodeHandle n("~");

    n.param<std::string>("line_topic", lineTopicName, "/line_detector/perception/line");
    n.param<std::string>("odometry_topic", odometryTopicName, "/odometry/filtered");
    n.param<std::string>("command_pub", commandPubName, "/fmCommand/cmd_vel");

    ros::Subscriber lineSub = n.subscribe(lineTopicName, 1, lineCb);
    ros::Subscriber enableSub = n.subscribe(odometryTopicName, 1, odometryCb);

    ros::Publisher commandPub = n.advertise<geometry_msgs::TwistStamped>(commandPubName, 1);
    ros::Rate rate(20);


    double start_yaw = tf::getYaw(odometry_info);

    while(ros::ok())
    {
        if(abs(start_yaw-tf::getYaw(odometry_info)) > 1.5)
        {
            break;
        }

        geometry_msgs::TwistStamped message;
        message.twist.angular.z = 1;
        commandPub.publish(message);

        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::TwistStamped message;
    message.twist.angular.z = 0;
    commandPub.publish(message);


    return 0;
}

void lineCb(const line_detection::line &msg)
{
    line_info = msg;
}

void odometryCb(const nav_msgs::Odometry &msg)
{
    tf::quaternionMsgToTF(msg.pose.pose.orientation, odometry_info);
}
