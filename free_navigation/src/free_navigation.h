#ifndef Navigation_H
#define Navigation_H
#include <ros/ros.h>
#include<string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<geometry_msgs/Pose.h>
#include <msgs/IntStamped.h>
#include <actionlib/server/simple_action_server.h>
#include <free_navigation/NavigateFreelyAction.h>
#include "line_pid/FollowLineAction.h"
using std::string;
using std::vector;
using geometry_msgs::Pose;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation
{
public:
    Navigation(std::string name);
private:
    void approachGoal();
    string name_;
    ros::NodeHandle nh_;

    // action server
    actionlib::SimpleActionServer<free_navigation::NavigateFreelyAction> as_;
    free_navigation::NavigateFreelyResult result_;
    uint8_t goal_;
    uint8_t prev_action_state_ = 255;
    // Move base client
    Pose line_to_manipulator_pose_;
    Pose delivery_pose_;
    Pose charge_pose_;
    Pose load_bricks_pose_;
    string static_frame_id;
    MoveBaseClient move_base_ac_;
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void goalCb();
    void preemtCb();
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    static Pose convertVecToPose(const vector<double>& poses);
    // Docking client
    actionlib::SimpleActionClient<line_pid::FollowLineAction> action_line_follow;
    void doneCbLine(const actionlib::SimpleClientGoalState& state,
                const line_pid::FollowLineResultConstPtr& result);
    line_pid::FollowLineGoal line_goal;
    const double stop_dist_before_tag = 0.25;
};
#endif // Navigation_H
