#include <ros/ros.h>
#include <mission/action_states.h>
#include <msgs/IntStamped.h>
#include <tf/tf.h>
#include <free_navigation/free_navigation.h>
using std::vector;
using std::string;
using geometry_msgs::Pose;

void actionCb(const msgs::IntStamped& cmd)
{
    ROS_INFO("Action: %i recieved", cmd.data);
}

int main(int argc, char** argv){
    ros::init(argc,argv,"navigator");
    ros::NodeHandle n;
    Navigation navigation;
    string action_topic;
    
    {
        string base_link_id;
        n.param<string>("base_link_id", base_link_id, "base_link");
        navigation.base_frame_id_ = base_link_id;
        n.param<string>("action_topic", action_topic, "/mission/action_state");
        vector<double> tmp_in_plane_pose;
        n.param<vector<double>>("line_to_manipulator", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.line_to_manipulator_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);

        n.param<vector<double>>("line_from_manipulator", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.line_from_manipulator_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
        n.param<vector<double>>("load_bricks", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.load_bricks_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
        n.param<vector<double>>("charge_initial", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.charge_initial_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
        n.param<vector<double>>("charge_dock", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.charge_dock_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
        n.param<vector<double>>("90degrees_left", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.turn_90_left_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
        n.param<vector<double>>("90degrees_right", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.turn_90_right_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
        n.param<vector<double>>("180degrees_turn", tmp_in_plane_pose, vector<double>{0,0,0});
        navigation.turn_180_pose_ = Navigation::convertVecToPose(tmp_in_plane_pose);
    }    
    ros::Subscriber sub = n.subscribe(action_topic, 1, &Navigation::actionStateCb, &navigation);
    //ros::Subscriber sub = n.subscribe(action_topic, 1, &actionCb);
    ros::spin();
    return 0;
}

