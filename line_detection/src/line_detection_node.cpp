#include <stdio.h>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <msgs/BoolStamped.h>
#include <line_detection/line.h>

using namespace cv;
using std::vector;
using std::string;

namespace line_types
{
    enum Line_type
    {
        NO_LINE,
        LINE,
        CROSS,
    };
}
typedef line_types::Line_type Line_type;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void camInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
inline bool advancedEdgeDetection(const Mat &image, Mat &edges);
inline Line_type scanline(const Mat& lineImage, int line, int &mid);
void verifyPubCb(const ros::TimerEvent &);

image_transport::Publisher image_pub;

ros::Publisher line_pub;
ros::Publisher verify_pub;

bool verification = false; /* Verify is set true when is likley that a "preception" is valid, */
int threshold_gray = 120;
int line_width = 60;
int cross_width = 300;
string camera_frame_id;
// TODO: read in camera matrix from file or camera
vector<double> cam_mat {1232.801045, 0, 640, 0, 1240.089791, 360, 0, 0, 1};
const double f_x = 1232.801045;
const double f_y = 1240.089791;
const double c_x = 640;
const double c_y = 360;
const Eigen::Vector2d cam_center(c_x, c_y);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n("~");
    n.param<int>("threshold_gray",threshold_gray, 120);
    n.param<int>("line_width", line_width, 60);
    n.param<int>("cross_width",cross_width, 300);
    int verify_pub_rate;
    n.param<int>("verification_pub_rate", verify_pub_rate, 5);
    string verification_topic;
    n.param<string>("verification_topic", verification_topic, "line_verify");
    string line_topic;
    n.param<string>("line_topic", line_topic, "line");    
    n.param<string>("camera_frame_id", camera_frame_id, "camera_link");
    bool show_line_enb;
    n.param<bool>("show_lines", show_line_enb, false);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/image_raw", 1, &imageCb);
    verify_pub = n.advertise<msgs::BoolStamped>(verification_topic,1);
    line_pub = n.advertise<line_detection::line>(line_topic,1);
    ros::Timer timeVerify = n.createTimer(ros::Duration(1.0 / verify_pub_rate), verifyPubCb);
    ros::spin();
    return 0;
}

void verifyPubCb(const ros::TimerEvent &) 
{
    ROS_INFO("timer");
    msgs::BoolStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.data = verification;
    verify_pub.publish(msg);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    try
    {
        cv_ptr =  cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    //search from top until a line without intersection is found or center of image
    int midOfTop, top_scanline=0;
    Line_type type_top = scanline(cv_ptr->image, top_scanline, midOfTop);
    while(type_top == line_types::CROSS && top_scanline < cv_ptr->image.rows)
    {
        top_scanline += 50;
        ROS_INFO("%i",top_scanline);
        type_top = scanline(cv_ptr->image, top_scanline, midOfTop);
    }

    //search from bottum until a line without intersection is found or center of image
    int midOfBot, bot_scanline = cv_ptr->image.rows-1;
    Line_type type_bot = scanline(cv_ptr->image, bot_scanline, midOfBot);
    //search until a line without intersection is found
    while(type_bot == line_types::CROSS && bot_scanline >= 0)
    {
        bot_scanline -= 50;
        type_bot = scanline(cv_ptr->image, bot_scanline, midOfBot);
    }

    // Set verification flag
    if(type_top == line_types::LINE && type_bot == line_types::LINE)
    {
        verification = true;
    } else {
        ROS_WARN("No valid line found");
    }

    // Determine angle
    double angle, offset; // zero error if no line found
    if(verification)
    {
        //
        Eigen::Vector2d pr0(midOfTop, top_scanline); Eigen::Vector2d pr1(midOfBot, bot_scanline);
        pr0 -= cam_center;
        pr1 -= cam_center;
        pr0[0] /= f_x; pr0[1] /= f_y;
        pr1[0] /= f_x; pr1[1] /= f_y;

       Eigen::ParametrizedLine<double, 2> param_line = Eigen::ParametrizedLine<double, 2>::Through(pr0, pr1);

        //Calculate line
        Eigen::Vector2d gradient = param_line.direction();
        double angle = atan2(gradient[1], gradient[0]) - CV_PI/2;

        Eigen::NumTraits<Scalar>::Real offset_real = param_line.distance(Eigen::Vector2d(0,0));

        double offset = offset_real[0];
        offset *=  (pr0[0] < 0 ? -1 : 1);
    }
    else {
        angle=0;
        offset=0;
    }

    // Publish line pose
    line_detection::line line_msg;
    line_msg.header.stamp = ros::Time::now();
    line_msg.header.frame_id = ""; // TODO:
    line_msg.angle = angle;
    line_msg.offset = offset;
    line_pub.publish(line_msg);

    // Debug plot ( TODO: move debug plot to other node )
    Mat line_img;
    threshold(cv_ptr->image, line_img,threshold_gray,255,0);
    cv::line(line_img, Point(midOfTop, top_scanline), Point(midOfBot, bot_scanline), 150, 2);
    cv::circle(line_img, Point(cv_ptr->image.cols/2+offset,0), 2, 100, 2);
    imshow("lines", line_img);
    cv::waitKey(1);
}


inline Line_type scanline(const Mat& lineImage, int line, int &mid)
{
    unsigned start, width, total_width = 0;
    mid = -1;
    bool counting = false;
    Line_type type = Line_type::NO_LINE;

    //scanline 1
    const uchar* p = lineImage.ptr<uchar>(line);
    for(unsigned x=0; x<lineImage.cols; x++)
    {
        //threshold to seek for line
        if(p[x] < threshold_gray)
        {
            //count the width of the found dark scan
            if(counting == false)
            {
                start = x;
                width = 1;
                counting = true;
            }
            else
            {
                width++;
                total_width++;
            }
        }
        //depending on length define what is found
        else if(counting == true)
        {
            counting = false;
            if(width > line_width) //this might happen twice and give a bug
            {
                type = Line_type::LINE;
                mid = start+width/2;

            }
            if(total_width > cross_width)
            {
                type = Line_type::CROSS;
                mid = start+width/2;
            }
        }
    }

    if(width > line_width) //this might happen twice and give a bug
    {
        type = Line_type::LINE;
        mid = start+width/2;

    }
    if(total_width > cross_width)
    {
        type = Line_type::CROSS;
        //mid = start+width/2;
    }
    return type;
}
