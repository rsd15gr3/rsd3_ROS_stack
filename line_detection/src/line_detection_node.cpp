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
Line_type scanline(Mat lineImage, int line, int &mid);

image_transport::Publisher image_pub;

ros::Publisher line_pub;
int loop_rate = 1;
bool verification = false; /* Verify is set true when is likley that a "preception" is valid, */
unsigned threshold_gray = 120;

vector<double> cam_mat {1232.801045, 0, 640, 0, 1240.089791, 360, 0, 0, 1};

const double f_x = 1232.801045;
const double f_y = 1240.089791;
const double c_x = 640;
const double c_y = 360;
const Eigen::Vector2d cam_center(c_x, c_y);

const unsigned LINE_WIDTH = 60;
const unsigned CROSS_WIDTH = 300;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n("~");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/usb_cam/image_raw", 1, &imageCb,image_transport::TransportHints("compressed"));
    //image_pub = it.advertise("/perception/floor_lines", 1);

    ros::Publisher verify_pub = n.advertise<msgs::BoolStamped>("perception/line_verify",1);
    line_pub = n.advertise<line_detection::line>("perception/line",1);

    ros::Rate r(loop_rate);

    while (ros::ok()) //Code loop
    {
        msgs::BoolStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.data = verification;

        verify_pub.publish(msg);
        r.sleep(); //sleep to match pub freq for testing
        ros::spinOnce();
    }
    ROS_INFO("line detector is dead!!!");
    return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    Mat lineImage;
    cvtColor(cv_ptr->image,lineImage,CV_BGR2GRAY);
    int midOfTop;
    int top_scanline = 0;
    Line_type type_top = scanline(lineImage, top_scanline, midOfTop);

    //search until a line without intersection is found
    while(type_top == line_types::CROSS)
    {
        top_scanline += 50;
        type_top = scanline(lineImage, top_scanline, midOfTop);
    }

    int midOfBot;
    int bot_scanline = lineImage.rows-1;
    Line_type type_bot = scanline(lineImage, bot_scanline, midOfBot);
    //search until a line without intersection is found
    while(type_bot == line_types::CROSS)
    {
        bot_scanline -= 50;
        type_bot = scanline(lineImage, bot_scanline, midOfBot);
    }

    if(type_top == line_types::LINE && type_bot == line_types::LINE)
    {
        verification = true;
    } else {
        ROS_INFO("No valid edge region of line found");
    }

    cv::line(lineImage, Point(midOfTop, top_scanline), Point(midOfBot, bot_scanline), 255, 2);
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
    offset *=  (pr0[0] > 0 ? 1 : -1);
    cv::circle(lineImage, Point(lineImage.cols/2+offset,0), 2, 255, 2);

    line_detection::line line_msg;
    line_msg.header.stamp = ros::Time::now();
    line_msg.header.frame_id = ""; // TODO:
    line_msg.angle = angle;
    line_msg.offset = offset;
    line_pub.publish(line_msg);
    imshow("slider", lineImage);
    cv::waitKey(5);
}


Line_type scanline(Mat lineImage, int line, int &mid)
{
    unsigned start, width, total_width = 0;
    mid = -1;
    bool counting = false;
    Line_type type = Line_type::NO_LINE;

    //scanline 1
    uchar* p;
    p = lineImage.ptr<uchar>(line);
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
            if(width > LINE_WIDTH) //this might happen twice and give a bug
            {
                type = Line_type::LINE;
                mid = start+width/2;

            }
            if(total_width > CROSS_WIDTH)
            {
                type = Line_type::CROSS;
                mid = start+width/2;
            }
        }
    }

    if(width > LINE_WIDTH) //this might happen twice and give a bug
    {
        type = Line_type::LINE;
        mid = start+width/2;

    }
    if(total_width > CROSS_WIDTH)
    {
        type = Line_type::CROSS;
        //mid = start+width/2;
    }
    return type;
}
