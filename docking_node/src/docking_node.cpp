#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/ransac.h>

laser_geometry::LaserProjection* projector_;
tf::TransformListener* listener_;

void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(!listener_->waitForTransform(msg->header.frame_id, "/base_link", msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    projector_->transformLaserScanToPointCloud("/base_link", *msg, cloud_msg, *listener_);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    std::vector<int> inliers;
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_c(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (cloud, true));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_c);
    ransac.setDistanceThreshold(.01);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    Eigen::VectorXf model_coefficients;
    model_c->computeModelCoefficients(inliers, model_coefficients);

    ROS_INFO_STREAM(model_coefficients(0) << " " << model_coefficients(1) << " " << model_coefficients(2));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "docking_node");
    ros::NodeHandle nh;

    listener_ = new tf::TransformListener;

    projector_ = new laser_geometry::LaserProjection;

    std::string laser_topic;
    nh.param("laser_topic", laser_topic, std::string("/fmSensors/scan"));
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>(laser_topic, 1, laserCB);
    ros::spin();

    delete listener_;
    delete projector_;

    return 0;
}
