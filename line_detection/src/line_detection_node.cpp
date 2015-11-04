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
#include <line_detection/cross.h>

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
        MULTIPLE_LINES,
    };
}
typedef line_types::Line_type Line_type;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void camInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
inline Line_type scanline(const Mat& lineImage, int line, double &threshold_gray, int &mid, bool cross = false);
void verifyPubCb(const ros::TimerEvent &);
double lineWidth(int line);
double sideLineWidth(int line);
bool findCross(line_detection::cross::Request  &req, line_detection::cross::Response &res);

image_transport::Publisher image_pub;
cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html

ros::Publisher line_pub;
ros::Publisher verify_pub;

bool verification = false; /* Verify is set true when is likley that a "preception" is valid, */
int threshold_gray = 105;
double threshold_top, threshold_bot;
double line_to_cross_scale = 3.0;
string camera_frame_id;
bool show_line_enb;
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

    n.param<int>("threshold_gray",threshold_gray, 105);
    threshold_top = threshold_gray; 
    threshold_bot = threshold_gray;
    n.param<double>("line_to_cross_scale",line_to_cross_scale, 3.0);
    int verify_pub_rate;
    n.param<int>("verification_pub_rate", verify_pub_rate, 5);
    string verification_topic;
    n.param<string>("verification_topic", verification_topic, "line_verify");
    string line_topic;
    n.param<string>("line_topic", line_topic, "/line_detector/perception/line");
    n.param<string>("camera_frame_id", camera_frame_id, "camera_link");    
    n.param<bool>("show_lines", show_line_enb, false);
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;
    image_sub = it.subscribe("/usb_cam/image_raw", 1, &imageCb);
    verify_pub = n.advertise<msgs::BoolStamped>(verification_topic,1);
    line_pub = n.advertise<line_detection::line>(line_topic,1);
    image_pub = it.advertise("line_debug_image", 1);
    ros::ServiceServer service = n.advertiseService("getCross", findCross);
    ros::Timer timeVerify = n.createTimer(ros::Duration(1.0 / verify_pub_rate), verifyPubCb);
    ros::spin();
    return 0;
}

bool findCross(line_detection::cross::Request &req,
               line_detection::cross::Response &res)
{
    int column_nr;
    if(req.right == 1)
    {
        column_nr = 0;
    }
    else
    {
        column_nr = cv_ptr->image.cols-2;
    }

    //Mask it as a row
    Mat column = cv_ptr->image(cv::Range::all(), cv::Range(column_nr,column_nr+1) );
    Mat line;
    cv::transpose(column, line);
    if(req.right == 0)
    {
        cv::flip(line,line, 0); // flip around x axis
    }

    int offset;
    double temp_threshold = threshold_gray;
    Line_type type = scanline(line, 0, temp_threshold, offset, true);
    res.offset = offset;

    if(type != line_types::LINE)
    {
        return false;
    }

    return true;
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
    // cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
    //moved up to share
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


    ROS_DEBUG("Threshold top: %f\n", threshold_top);
    Line_type type_top = scanline(cv_ptr->image, top_scanline, threshold_top, midOfTop);
    while(type_top == line_types::CROSS && top_scanline < cv_ptr->image.rows)
    {
        top_scanline += 50;
        double temp = threshold_top;
        type_top = scanline(cv_ptr->image, top_scanline, threshold_top, midOfTop);
        threshold_top = temp;
    }

    //search from bottum until a line without intersection is found or center of image
    int midOfBot, bot_scanline = cv_ptr->image.rows-1;
    ROS_DEBUG("Threshold bot: %f\n", threshold_bot);
    Line_type type_bot = scanline(cv_ptr->image, bot_scanline, threshold_bot, midOfBot);
    //search until a line without intersection is found
    while(type_bot == line_types::CROSS && bot_scanline >= 0)
    {
        bot_scanline -= 50;
        double temp = threshold_bot;
        type_bot = scanline(cv_ptr->image, bot_scanline, threshold_bot, midOfBot);
        threshold_bot = temp;
    }

    // Determine angle
    double angle, offset; // zero error if no line found
    if(type_top == line_types::LINE && type_bot == line_types::LINE)
    {
        verification = true;
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
        threshold_top = threshold_gray;
        threshold_bot = threshold_gray;
        ROS_WARN("No valid line found");
        angle=0;
        offset=0;
    }    
    if(show_line_enb)
    {
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
        //imshow("lines top", line_img);

        threshold(cv_ptr->image, line_img,threshold_bot,255,0);
        cv::line(line_img, Point(midOfTop, top_scanline), Point(midOfBot, bot_scanline), 255, 2);
        //imshow("lines bot", line_img);

        //cv::waitKey(1);
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", line_img).toImageMsg();
        image_pub.publish(msg);
    }
}

inline Line_type scanline(const Mat& lineImage, int line, double &threshold_gray, int &mid, bool cross)
{
    unsigned int start, width, total_width = 0;
    double average_white = 0;
    double average_black = 0;
    mid = -1;
    bool counting = false;
    Line_type type = Line_type::NO_LINE;

    //assign different width if it is crossing detection
    double (*fp_linewidth)(int);
    fp_linewidth = lineWidth;
    if(cross == true)
    {
        fp_linewidth = sideLineWidth;
    }

    //scanline 1
    const uchar* p;
    p = lineImage.ptr<uchar>(line);
    for(unsigned x=0; x<lineImage.cols; x++)
    {
        //threshold to seek for line
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
                if(width > fp_linewidth(line)) //this might happen twice and give a bug
                {
                    type = Line_type::LINE;
                    mid = start+width/2;
                }
                if(total_width > (fp_linewidth(line)*line_to_cross_scale+10))
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

    //103 190

    return type;
}

//gives the line width depending on the position in the image
double lineWidth(int line)
{
    return 190.0-(190.0-103.0)/720.0*(double)line-10.0;
}

double sideLineWidth(int line)
{
    return 100;
}
