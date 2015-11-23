#ifndef Navigation_H
#define Navigation_H
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <msgs/IntStamped.h>
#include <relative_move_server/RelativeMoveAction.h>
using std::string;
using std::vector;
using geometry_msgs::Pose;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef actionlib::SimpleActionClient<relative_move_server::RelativeMoveAction> ChargingClient;


class Navigation
{
public:
    Navigation();
    void setNavigationPoint(int action_state_id, vector<double> in_plane_pose);
    void actionStateCb(const msgs::IntStamped& action_state);
    bool setTargetWayPoint(const geometry_msgs::Pose goalPose);
    static Pose convertVecToPose(const vector<double>& poses);
    //Waypoints for absolute navigation poses
    Pose line_to_manipulator_pose_;
    Pose line_from_manipulator_pose_;
    Pose load_bricks_pose_;
    Pose charge_initial_pose_;
    Pose charge_dock_pose_;
    //Relative movement poses
    Pose turn_180_pose_;
    Pose turn_90_right_pose_;
    Pose turn_90_left_pose_;
    string base_frame_id_;
private:
    MoveBaseClient ac_;
    ChargingClient char_client_;
    int prev_action_state_ = -1;

    bool moving_;
    bool rel_move_done_;

    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    void doneRelativeMovCb(const actionlib::SimpleClientGoalState& state, const relative_move_server::RelativeMoveResultConstPtr& result);
    void activeRelativeMovCb();
    void feedbackRelativeMovCb(const relative_move_server::RelativeMoveFeedbackConstPtr& feedback);
    void setBaseFrameId(string frame_id);

    bool setRelativeMove(double dx, double dy, double dth);
    bool sendGoal(relative_move_server::RelativeMoveGoal& goal);
};

#endif // Navigation_H
