#include <stdio.h>
#include <ros/ros.h>
#include <brick_detection/bricks.h>

#include <msgs/BoolStamped.h>

#include <cv.h>
#include <highgui.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <string>
#include <mutex>


using namespace cv;
using std::vector;
using std::string;

Mat newest_image;
std::mutex mtx;

void imageCb(const sensor_msgs::ImageConstPtr& msg);

image_transport::Publisher image_pub;

string camera_frame_id;

bool findBricks(brick_detection::bricks::Request  &req,
                brick_detection::bricks::Response &res)
{
    ROS_INFO("got a request");

    mtx.lock();
    //Rect ROI(400 ,290,image.cols-1-800,image.rows-490);
    //Mat img = image(ROI);
    //detectBrick(img);

    //cv::imshow("test", newest_image);
    //cv::waitKey(1);

    //res.angle.resize(1);
    for(int i=0; i<1; i++)
    {
        res.x.push_back(1);
        res.y.push_back(2);
        res.angle.push_back(3);
        res.type.push_back(0);
    }

    mtx.unlock();
    cv::imshow("test1", newest_image);

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "brick_detector_servor");
    ros::NodeHandle n("~");

    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/usb_cam/image_raw", 1, &imageCb);

    ros::ServiceServer service = n.advertiseService("getBricks", findBricks);

    ros::spin();
    return 0;
}


void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    try
    {
        cv_ptr =  cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mtx.lock();
    newest_image = cv_ptr->image.clone();
    cv::waitKey(1);
    mtx.unlock();

    //cv::imshow("test2", newest_image);
}

