#ifndef Navigation_H
#define Navigation_H
#include<string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include<geometry_msgs/Pose.h>
#include <msgs/IntStamped.h>
using std::string;
using std::vector;
using geometry_msgs::Pose;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Navigation
{
public:
    Navigation();
    void setNavigationPoint(int action_state_id, vector<double> in_plane_pose);
    void actionStateCb(const msgs::IntStamped& action_state);
    static Pose convertVecToPose(const vector<double>& poses);
    Pose line_to_manipulator_pose_;
    Pose line_from_manipulator_pose_;
    Pose charge_pose_;
    Pose load_bricks_pose_;
    string base_frame_id_;
private:
    MoveBaseClient ac_;
    int prev_action_state_ = -1;
    void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);
    void activeCb();
    void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

    void setBaseFrameId(string frame_id);
};

#endif // Navigation_H
