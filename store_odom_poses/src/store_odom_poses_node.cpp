#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
using namespace std;

void filteredPoseCb(const geometry_msgs::PoseWithCovarianceStamped &filtered_pose);
void odomPoseCb(const nav_msgs::Odometry &odom_pose);
ofstream filtered_position_file;
ofstream odom_position_file;
int main(int argc, char** argv)
{
  ros::init(argc,argv,"navigator");
  ros::NodeHandle n;
  /*
  if(argc < 3)
  {
    std::cout << "Wrong numer of input arguments" << std::endl;
    std::cout << "arg: path/odom_pose_filename path/filtered_pose_filename " << std::endl;
  }
  */
  string odom_pose_filename("/home/anders/Documents/odom_poses.txt");
  string filtered_filename("/home/anders/Documents/filtered_poses.txt");
  string header_str("time_stamp,x,y\n");
  odom_position_file.open (odom_pose_filename.c_str());
  odom_position_file << header_str.c_str();
  filtered_position_file.open (filtered_filename.c_str());
  filtered_position_file << header_str.c_str() ;
  ros::Subscriber odom_sub = n.subscribe("/fmKnowledge/wheel_odom", 100, &odomPoseCb);
  ros::Subscriber filtered_sub = n.subscribe("/fmProcessors/robot_pose_ekf/odom_combined", 100, &filteredPoseCb);
  ros::spin();
  odom_position_file.close();
  filtered_position_file.close();
  std::cout << "After spin" << std::endl;
}

void filteredPoseCb(const geometry_msgs::PoseWithCovarianceStamped  &filtered_pose)
{
  filtered_position_file
    << filtered_pose.header.stamp << ","
    << filtered_pose.pose.pose.position.x << ","
    << filtered_pose.pose.pose.position.y << std::endl;
}

void odomPoseCb(const nav_msgs::Odometry &odom_pose)
{
  odom_position_file
    << odom_pose.header.stamp << ","
    << odom_pose.pose.pose.position.x << ","
    << odom_pose.pose.pose.position.y << std::endl;
}
