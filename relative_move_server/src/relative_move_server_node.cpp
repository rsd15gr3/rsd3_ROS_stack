#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "tf/transform_broadcaster.h"
#include <nav_msgs/Odometry.h>

#include <actionlib/server/simple_action_server.h>
#include <relative_move_server/RelativeMoveAction.h>

#include <angles/angles.h>

class RelativeMoveAction{
public:
    RelativeMoveAction(ros::NodeHandle& nh): nh_(&nh), as_(nh, "relative_move_action", boost::bind(&RelativeMoveAction::executeCB, this, _1), false){
        std::string cmd_vel_topic;
        nh_->param("cmd_vel_topic", cmd_vel_topic, std::string("/cmd_vel"));
        cmd_vel_pub_ = nh_->advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);

        std::string odom_topic;
        nh_->param("odom_topic", odom_topic, std::string("/odom"));
        odom_sub_ = nh_->subscribe(odom_topic, 1, &RelativeMoveAction::odomCB, this);

        nh_->param("pid_dist_offset", pid_dist_offset_, 0.85);
        nh_->param("/fmExecutors/wptnav/max_linear_velocity", max_lin_, 1.2);
        nh_->param("/fmExecutors/wptnav/max_angular_velocity", max_rot_, 0.9);
        nh_->param("/fmExecutors/wptnav/drive_kp", Kp_, 10.0);
        nh_->param("/fmExecutors/wptnav/drive_ki", Ki_, 0.0);
        nh_->param("/fmExecutors/wptnav/drive_kd", Kd_, 0.0);

        done_=false;
        failed_=false;
        tf_listener_= new tf::TransformListener();

        as_.start();
    }

private:
    void publishCmdVel(double lin, double rot){
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = lin;
        cmd_vel.angular.z = rot;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void sendFeedback(double x, double y, double th){
        relative_move_server::RelativeMoveFeedback feedback;

        feedback.dist_to_goal.pose.position.x = x;
        feedback.dist_to_goal.pose.position.y = y;
        feedback.dist_to_goal.pose.orientation = tf::createQuaternionMsgFromYaw(th);

        //feedback.current_goal = ;

        as_.publishFeedback(feedback);
    }

    bool testQuaternion(geometry_msgs::Quaternion quaternion){
        double total;
        total = quaternion.x + quaternion.y + quaternion.z + quaternion.w;
        if(fabs(total-1) > 0.01){
            return false;
        }
        else{
            return true;
        }
    }

    void odomCB(const nav_msgs::Odometry::ConstPtr& msg){
        base_odom_.header.frame_id = msg->header.frame_id;
        base_odom_.header.stamp = msg->header.stamp;
        base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
        base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
        base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;

        base_odom_.pose.pose.position.x = msg->pose.pose.position.x;
        base_odom_.pose.pose.position.y = msg->pose.pose.position.y;
        base_odom_.pose.pose.orientation = msg->pose.pose.orientation;

        if(moving_){
            performRelativeMove(msg->header.stamp);
        }
    }

    bool checkGoal(relative_move_server::RelativeMoveGoal& goal){
        if(fabs(goal.target_pose.pose.position.x)>1e-4 || fabs(goal.target_pose.pose.position.y)>1e-4 || fabs(tf::getYaw(goal.target_pose.pose.orientation))>1e-4){
            if(fabs(goal.target_yaw)>1e-4){
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal.target_yaw);
            }

            if(testQuaternion(goal.target_pose.pose.orientation)){
                ROS_WARN_STREAM("Wrong quaternion in the goal. Rotation set to 0.");
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
            }

            try{
                tf_listener_->waitForTransform(base_odom_.header.frame_id, goal.target_pose.header.frame_id, goal.target_pose.header.stamp, ros::Duration(1.0));
                tf_listener_->transformPose(base_odom_.header.frame_id, goal.target_pose, relative_move_pose_);
            }catch(tf::LookupException& e){
                ROS_ERROR("No transform available. Error: %s\n", e.what());
                return false;
            }catch(tf::ConnectivityException& e){
                ROS_ERROR("Connectivity error: %s\n", e.what());
                return false;
            }catch(tf::ExtrapolationException& e){
                ROS_ERROR("Extrapolation error: %s\n", e.what());
                return false;
            }

            double dx = relative_move_pose_.pose.position.x - base_odom_.pose.pose.position.x;
            double dy = relative_move_pose_.pose.position.y - base_odom_.pose.pose.position.y;
            double yaw = tf::getYaw(base_odom_.pose.pose.orientation);
            double dist = sqrt(pow(dx, 2) + pow(dy, 2));

            if(dist < 0.02){
                pure_rotation_ = true;
            }
            else{
                pure_rotation_ = false;
            }

            double dot = dx*cos(yaw) + dy*sin(yaw);

            if(dot < 0){
                forward_ = false;
            }
            else{
                forward_ = true;
            }

            moving_ = true;
            failed_ = false;
            done_ = false;
        }
        else{
            ROS_ERROR_STREAM("Too small relative move.");
            return false;
        }

        return true;
    }

    bool performRelativeMove(ros::Time time_stamp){
        static ros::Time last_update = ros::Time::now();
        static double last_crosstrack_error = 1e9;
        static double summed_crosstrack_error = 0;

        double lin = 0;
        double rot = 0;
        double dt = fmax(0.1, (time_stamp - last_update).toSec());

        double dx = relative_move_pose_.pose.position.x - base_odom_.pose.pose.position.x;
        double dy = relative_move_pose_.pose.position.y - base_odom_.pose.pose.position.y;
        double yaw = tf::getYaw(base_odom_.pose.pose.orientation);
        double goal_yaw = tf::getYaw(relative_move_pose_.pose.orientation);
        double dTh = angles::shortest_angular_distance(yaw, goal_yaw);
        double dist = sqrt(pow(dx, 2) + pow(dy, 2));

        sendFeedback(dx, dy, dTh);

        double dot = dx*cos(yaw)+dy*sin(yaw);

        bool past_goal = false;

        if(!pure_rotation_){
            double x0 = relative_move_pose_.pose.position.x;
            double y0 = relative_move_pose_.pose.position.y;
            double x1 = relative_move_pose_.pose.position.x + cos(goal_yaw);
            double y1 = relative_move_pose_.pose.position.y + sin(goal_yaw);

            double pid_offset = pid_dist_offset_;
            if(!forward_){
                pid_offset = -pid_offset;
            }

            double xR = base_odom_.pose.pose.position.x + cos(yaw)*pid_offset;
            double yR = base_odom_.pose.pose.position.y + sin(yaw)*pid_offset;

            double crosstrack_error = ((y1-y0)*xR - (x1-x0)*yR + x1*y0 - y1*x0)/(sqrt(pow(y1-y0, 2) + pow(x1-x0, 2)));

            double velocity = fmin(max_lin_, fmax(0.1, dist));

            if(last_crosstrack_error > 1e8){
                last_crosstrack_error = crosstrack_error;
            }

            summed_crosstrack_error += crosstrack_error;

            double diff_crosstrack = (crosstrack_error - last_crosstrack_error)/dt;
            last_crosstrack_error = crosstrack_error;

            double control = (Kp_*crosstrack_error + Ki_*summed_crosstrack_error + Kd_*diff_crosstrack) * velocity;

            if(forward_){
                lin = velocity;
                rot = control;
            }
            else{
                lin = -velocity;
                rot = -control;
            }

            if(fabs(control)>max_rot_){
                lin /= fabs(control)/max_rot_;
                rot /= fabs(control)/max_rot_;
            }
            ROS_INFO_STREAM("Error: " << crosstrack_error << std::endl << " dx: " << dx << " dy: " << dy << " dTh: " << dTh << " Dist: " << dist << std::endl << "Lin: " << lin << " Ang: " << rot);
        }
        else{
            double v_theta = dTh > 0 ? fmin(max_rot_, fmax(0.1, dTh)) : fmax(-1.0*max_rot_, fmin(-0.1, dTh));
            rot = v_theta;
            ROS_INFO_STREAM("Pure rotation. Vth: " << v_theta);
        }

        if(forward_ && dot<0){
            ROS_INFO("Moved forward past goal.");
            past_goal = true;
        }
        if(!forward_ && dot>0){
            ROS_INFO("Moved backwards past goal.");
            past_goal = true;
        }

        if(past_goal && !pure_rotation_){
            ROS_INFO_STREAM("Drived past goal.");
            moving_ = false;
            pure_rotation_ = false;
            done_ = true;
            lin = 0;
            rot = 0;
        }
        else if((fabs(dTh) < 3*M_PI/180.0) && (dist < 0.02 || pure_rotation_)){
            double dtm = dt;
            bool keep_driving = false;
            if(lin != 0){
                double nextdx = relative_move_pose_.pose.position.x - (base_odom_.pose.pose.position.x + cos(yaw) * lin * dtm);
                double nextdy = relative_move_pose_.pose.position.y - (base_odom_.pose.pose.position.y + sin(yaw) * lin * dtm);
                double newdist = sqrt(pow(nextdx, 2) + pow(nextdy, 2));
                if(newdist < dist){
                    keep_driving = true;
                }
                else{
                    lin = 0;
                }
            }

            bool keep_turning = false;
            if(!keep_driving && fabs(rot) < 0.1){
                rot = copysign(0.1, rot);
            }
            double newyaw = yaw + rot*dtm;
            if(fabs(angles::shortest_angular_distance(newyaw, goal_yaw))<fabs(dTh)){
                keep_turning = true;
            }
            else{
                rot = 0;
            }

            if(keep_driving || keep_turning){
                ROS_INFO_STREAM("Close to goal but continuing.");
            }
            else{
                ROS_INFO_STREAM("Goal reached. Dist: " << dist << " Th: " << dTh);
                moving_ = false;
                pure_rotation_ = false;
                done_ = true;
                last_crosstrack_error = 1e9;
                summed_crosstrack_error = 0;
            }
        }

        publishCmdVel(lin, rot);
        last_update = time_stamp;

        return true;
    }

    void executeCB(const relative_move_server::RelativeMoveGoalConstPtr &goal){
        relative_move_server::RelativeMoveGoal currentGoal = *goal;

        if(!checkGoal(currentGoal)){
            relative_move_server::RelativeMoveResult relative_move_result;
            relative_move_result.end_state = relative_move_server::RelativeMoveResult::INVALID_GOAL;
        }

        while(nh_->ok()){
            if(as_.isPreemptRequested()){
                if(as_.isNewGoalAvailable()){
                    currentGoal = *as_.acceptNewGoal();

                    if(!checkGoal(currentGoal)){
                        relative_move_server::RelativeMoveResult relative_move_result;
                        relative_move_result.end_state = relative_move_server::RelativeMoveResult::INVALID_GOAL;
                        as_.setAborted(relative_move_result);
                        publishCmdVel(0, 0);
                        return ;
                    }
                }
            }
            else{
                as_.setPreempted();
                moving_ = false;
                publishCmdVel(0, 0);
                return ;
            }

            if(!moving_){
                if(failed_){
                    as_.setAborted(relative_move_server::RelativeMoveResult(), "Relative move failed.");
                    failed_ = false;
                    return ;
                }

                if(done_){
                    as_.setSucceeded(relative_move_server::RelativeMoveResult(), "Relative move succeded.");
                    done_ = false;
                    return ;
                }
            }

            ros::WallDuration(0.01).sleep();
        }

        as_.setAborted(relative_move_server::RelativeMoveResult(), "Aborting because the node has been killed.");
    }

    ros::NodeHandle *nh_;

    actionlib::SimpleActionServer<relative_move_server::RelativeMoveAction> as_;

    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;

    nav_msgs::Odometry base_odom_;
    geometry_msgs::PoseStamped relative_move_pose_;

    tf::TransformListener* tf_listener_;

    double pid_dist_offset_;

    double Kp_, Ki_, Kd_;
    double max_lin_, max_rot_;

    bool moving_;
    bool pure_rotation_;
    bool forward_;
    bool done_;
    bool failed_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "relative_move_node");
    ros::NodeHandle n("~");
    RelativeMoveAction RelativeMoveActionNode(n);
    ros::spin();

    return 0;
}
