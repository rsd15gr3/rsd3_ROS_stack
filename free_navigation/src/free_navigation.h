#ifndef Navigation_H
#define Navigation_H
#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <msgs/IntStamped.h>
#include <actionlib/server/simple_action_server.h>
#include "free_navigation/NavigateFreelyAction.h"
//#include "dock_with_tape/DockWithTapeAction.h"
#include <relative_move_server/RelativeMoveAction.h>
#include <collect_bricks_pos/collect_bricks_posAction.h>
#include <docking_with_walls/docking_with_wallsAction.h>

using std::string;
using std::vector;
using geometry_msgs::Pose;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation
{
public:
    Navigation(std::string name);
private:
    enum position_state
    {
      docked, under_dispenser, free, at_transition
    } current_position;
    void approachGoal();
    string name_;
    ros::NodeHandle nh_;
    // action server
    actionlib::SimpleActionServer<free_navigation::NavigateFreelyAction> as_;
    free_navigation::NavigateFreelyResult result_;
    uint8_t goal_;
    uint8_t prev_action_state_ = 255;
    // Move base client
    Pose line_to_manipulator_pose_, delivery_pose_, charge_pose_;
    Pose load_bricks_pose_, recovery_;
    string static_frame_id;
    MoveBaseClient move_base_ac_;
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);    
    void goalCb();
    void preemtCb();
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    static Pose convertVecToPose(const vector<double>& poses);
    void sendMoveBaseGoal(move_base_msgs::MoveBaseGoal& goal_msg);
    // Docking client
    actionlib::SimpleActionClient<docking_with_walls::docking_with_wallsAction> docking_with_walls_ac_;
    void doneCbWalls(const actionlib::SimpleClientGoalState& state, const docking_with_walls::docking_with_wallsResultConstPtr& result);
    // relative move
    void doneRelativeMoveCb(const actionlib::SimpleClientGoalState& state, const relative_move_server::RelativeMoveResultConstPtr& result);
    actionlib::SimpleActionClient<relative_move_server::RelativeMoveAction> relative_move_ac_;
    relative_move_server::RelativeMoveGoal getRelativeMove(double dx, double dy, double dth);
    double undock_relative_move;
    bool in_recovery_mode;
    // collect client
    actionlib::SimpleActionClient<collect_bricks_pos::collect_bricks_posAction> collect_bricks_ac_;
    void doneCollectBricksCv(const actionlib::SimpleClientGoalState& state, const collect_bricks_pos::collect_bricks_posResultConstPtr& result);

};
#endif // Navigation_H
