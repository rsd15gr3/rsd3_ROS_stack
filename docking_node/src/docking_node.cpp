#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <laser_geometry/laser_geometry.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/filters/passthrough.h>

laser_geometry::LaserProjection* projector_;
tf::TransformListener* listener_;
ros::Publisher marker_pub_, inlier_pub_, pcl_pub_;

void laserCB(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(!listener_->waitForTransform(msg->header.frame_id, "/base_link", msg->header.stamp + ros::Duration().fromSec(msg->ranges.size()*msg->time_increment), ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud2 cloud_msg;
    projector_->transformLaserScanToPointCloud("/laser_link", *msg, cloud_msg, *listener_);
    //pcl_pub_.publish(cloud_msg);

    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(cloud_msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (0.1, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud);

    pcl_pub_.publish(cloud);

    std::vector<int> inliers;
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_c(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (cloud, true));
    //model_c->setRadiusLimits(0.01, 0.2);
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_c);
    ransac.setDistanceThreshold(0.01);
    ransac.setMaxIterations(10000);
    ransac.computeModel();
    ransac.getInliers(inliers);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

    inlier_pub_.publish(final);

    Eigen::VectorXf model_coefficients, first;
    ransac.getModelCoefficients(first);
    //model_c->computeModelCoefficients(inliers, model_coefficients);

    model_c->optimizeModelCoefficients(inliers, first, model_coefficients);

    visualization_msgs::Marker marker;

    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "circle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = model_coefficients(0);
    marker.pose.position.y = model_coefficients(1);
    marker.pose.position.z = 0.2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = model_coefficients(2);
    marker.scale.y = model_coefficients(2);
    marker.scale.z = model_coefficients(2);
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;

    marker.lifetime = ros::Duration();

    marker_pub_.publish(marker);

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

    marker_pub_ = nh.advertise<visualization_msgs::Marker>("sphere", 1);
    pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
    inlier_pub_ = nh.advertise<sensor_msgs::PointCloud2>("inliers", 1);

    ros::spin();

    delete listener_;
    delete projector_;

    return 0;
}
