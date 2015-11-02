#include <stdio.h>
#include <vector>
#include <list>
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
inline Line_type scanline(const Mat& lineImage, int line, double &threshold_gray, int &mid);
void verifyPubCb(const ros::TimerEvent &);
void getRow(Mat Image, Mat &lineImage, int line);
void getColumn(Mat Image, Mat &lineImage, int line);
double lineWidth(int line);
uchar runningAverage(uchar value);

image_transport::Publisher image_pub;

ros::Publisher line_pub;
ros::Publisher verify_pub;

bool verification = false;
double threshold_top, threshold_bot;
int line_width, cross_width, threshold_gray;
std::list<uchar> average;

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
    threshold_top = threshold_gray; 
    threshold_bot = threshold_gray;
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
    image_sub = it.subscribe("/usb_cam/image_raw", 1, &imageCb);
    verify_pub = n.advertise<msgs::BoolStamped>(verification_topic,1);
    line_pub = n.advertise<line_detection::line>(line_topic,1);
    ros::Timer timeVerify = n.createTimer(ros::Duration(1.0 / verify_pub_rate), verifyPubCb);
    ros::spin();
    return 0;
}

void verifyPubCb(const ros::TimerEvent &) 
{
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

    //search from top until a line without intersection is found
    int midOfTop, top_scanline=0;
    Mat line;
    getRow(cv_ptr->image, line, top_scanline);
    ROS_INFO("Threshold top: %f\n", threshold_top);
    Line_type type_top = scanline(line, top_scanline, threshold_top, midOfTop);
    //search until a line without intersection is found
    while(type_top == line_types::CROSS && top_scanline < cv_ptr->image.rows)
    {
        top_scanline += 50;
        getRow(cv_ptr->image, line, top_scanline);
        double temp = threshold_top;
        type_top = scanline(line, top_scanline, threshold_top, midOfTop);
        threshold_top = temp;
    }

    //search from bottum until a line without intersection is found
    int midOfBot, bot_scanline = cv_ptr->image.rows-1;
    getRow(cv_ptr->image, line, bot_scanline);
    ROS_INFO("Threshold bot: %f\n", threshold_bot);
    Line_type type_bot = scanline(line, bot_scanline, threshold_bot, midOfBot);
    //search until a line without intersection is found
    while(type_bot == line_types::CROSS && bot_scanline >= 0)
    {
        bot_scanline -= 50;
        double temp = threshold_bot;
        type_bot = scanline(line, bot_scanline, threshold_bot, midOfBot);
        threshold_bot = temp;
    }

    // Determine angle
    double angle, offset; // zero error if no line found
    if(type_top == line_types::LINE && type_bot == line_types::LINE)
    {
        verification = true;
        //
        Eigen::Vector2d pr0(midOfTop, top_scanline); Eigen::Vector2d pr1(midOfBot, bot_scanline);
        pr0 -= cam_center;
        pr1 -= cam_center;
        pr0[0] /= f_x; pr0[1] /= f_y;
        pr1[0] /= f_x; pr1[1] /= f_y;

       Eigen::ParametrizedLine<double, 2> param_line = Eigen::ParametrizedLine<double, 2>::Through(pr0, pr1);

        //Calculate line
        Eigen::Vector2d gradient = param_line.direction();
        angle = atan2(gradient[1], gradient[0]) - CV_PI/2;

        Eigen::NumTraits<Scalar>::Real offset_real = param_line.distance(Eigen::Vector2d(0,0));

        offset = offset_real[0];
        offset *=  (pr0[0] < 0 ? -1 : 1);
    }
    else
    {
        verification = false;
        threshold_bot = threshold_gray;
        threshold_top = threshold_gray;

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
    cv::line(line_img, Point(midOfTop, top_scanline), Point(midOfBot, bot_scanline), 255, 2);

    threshold(cv_ptr->image, line_img,threshold_top,255,0);
    cv::line(line_img, Point(midOfTop, top_scanline), Point(midOfBot, bot_scanline), 255, 2);
    //cv::circle(line_img, Point(cv_ptr->image.cols/2+offset,0), 2, 100, 2);
    imshow("lines top", line_img);

    threshold(cv_ptr->image, line_img,threshold_bot,255,0);
    cv::line(line_img, Point(midOfTop, top_scanline), Point(midOfBot, bot_scanline), 255, 2);
    //cv::circle(line_img, Point(cv_ptr->image.cols/2+offset,0), 2, 100, 2);
    imshow("lines bot", line_img);
    imshow("line", line);

    cv::waitKey(1);
}

void getRow(Mat Image, Mat &lineImage, int line)
{
    lineImage = Image(cv::Range(line,line+1), cv::Range::all());
}

void getColumn(Mat Image, Mat &lineImage, int line)
{
    lineImage = Image(cv::Range::all(), cv::Range(line,line+1));
    cv::transpose(lineImage, lineImage);
}

inline Line_type scanline(const Mat& lineImage, int line, double &threshold_gray, int &mid)
{
    unsigned int start, width, total_width = 0;
    double average_white = 0;
    double average_black = 0;
    mid = -1;
    bool counting = false;
    Line_type type = Line_type::NO_LINE;

    const uchar* p;
    p = lineImage.ptr<uchar>(0);
    for(int x=0; x<lineImage.cols; x++)
    {
        //uchar avg = runningAverage(p[x]);
        //ROS_INFO("value: %d\n", (int)p[x]);
        //cv::waitKey();

        if(p[x] < threshold_gray)
        {
            average_black += p[x];
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
        if(p[x] > threshold_gray || x == lineImage.cols-1)
        {
            average_white += p[x];
            if(counting == true)
            {
                counting = false;
                if(width > lineWidth(line)) //this might happen twice and give a bug
                {
                    type = Line_type::LINE;
                    mid = start+width/2;
                }
                if(total_width > (lineWidth(line)*2+10))
                {
                    type = Line_type::CROSS;
                }
            }
        }
    }

    //update threshold
    if(type == Line_type::LINE)
    {
        threshold_gray = (average_black/total_width+average_white/(lineImage.cols-total_width))/2;
    }

    //if(total_width > )

    return type;
}

//gives the line width depending on the position in the image
double lineWidth(int line)
{
    return 190.0-(190.0-103.0)/720.0*(double)line-10.0;
}

const int average_size_limit = 5;
uchar runningAverage(uchar value)
{
    average.push_front(value);
    if(average.size() > average_size_limit)
    {
        average.pop_back();
    }
    int sum = 0;
    for(std::list<uchar>::iterator i=average.begin(); i != average.end(); i++)
    {
        sum += (int)*i;
    }
    sum /= average_size_limit;

    return (uchar)sum;
}
