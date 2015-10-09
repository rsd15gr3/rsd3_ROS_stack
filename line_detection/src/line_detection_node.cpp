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

using namespace cv;
using std::vector;
using std::string;

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void camInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg);
inline bool advancedEdgeDetection(const Mat &image, Mat &edges);
image_transport::Publisher image_pub;

int loopRate = 20;
bool verify = false; /* Verify is set true when is likley that a "preception" is valid, */

const double radPerDeg = CV_PI/180;
// Canny
const unsigned low_threshold = 100;
const unsigned low_to_high_ration = 4; // 2 or 3
const unsigned kernel_size = 3;
// Advanced edge detection
const unsigned MIN_COUNTOUR_AREA = 20000; // PIXELS
const unsigned SECTION_SIZE = 80; // PIXELS
// Probablistic Hough
const unsigned rho = 2;
const unsigned theta = 2;
const unsigned min_no_of_intersections = 70;
const unsigned min_line_length = 150;
const unsigned max_line_gap = 20;
vector<double> cam_mat {1232.801045, 0, 643.489543, 0, 1240.089791, 375.302479, 0, 0, 1};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_detector");
    ros::NodeHandle n("~");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber image_sub;

    image_sub = it.subscribe("/usb_cam/image_raw", 1, &imageCb);
    image_pub = it.advertise("/perception/floor_lines", 1);

    ros::Publisher pub = n.advertise<msgs::BoolStamped>("perception/example_verify_pub",1);
    ros::Rate loop_rate(loopRate);

    while (ros::ok()) //Code loop
    {
        verify = !verify;

        msgs::BoolStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.data = verify;

        pub.publish(msg);

        loop_rate.sleep(); //sleep to match pub freq for testing
        ros::spinOnce();
    }

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
    Mat edges;
    // Canny(cv_ptr->image, edges, low_threshold, low_threshold*low_to_high_ration, kernel_size);
    bool valid_line = advancedEdgeDetection(cv_ptr->image, edges);
    imshow("edges of line", edges);
    cv::waitKey(5);
    if(valid_line) {
        vector<Vec4i> lines;
        HoughLinesP(edges, lines, rho, theta*radPerDeg, min_no_of_intersections, min_line_length, max_line_gap);
        // Run through lines and parametrize
        vector<Eigen::ParametrizedLine<double, 2> > param_lines(lines.size());
        ROS_INFO("Found %i lines",(int)lines.size());
        for( size_t i = 0; i < lines.size(); i++ )
        {
          Vec4i l = lines[i];
          line( cv_ptr->image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA); // plot lines
          // convert to line to parametrized line
          Eigen::Vector2d pr0(l[0], l[1]); Eigen::Vector2d pr1(l[2], l[3]);
          param_lines[i] = Eigen::ParametrizedLine<double, 2>::Through(pr0, pr1);
          //ROS_INFO("Origin = %f", param_lines[i].origin()[0]);
        }
        /*
        // combine lines
        // atan2(a/1)
        // calculate error from center http://eigen.tuxfamily.org/dox/classEigen_1_1ParametrizedLine.html
        l.origin();
        double dist = l.distance()
        */
        image_pub.publish(cv_ptr->toImageMsg());
        verify = (lines.size() >= 2); // TODO: improve verify
    }
    else {
        ROS_INFO("No valid edge region of line found");
    }
}

inline void discrete_threshold(const Mat &image, Mat &binaryIm, int thresholdGray, int light)
{
    //convert to gray
    Mat gray;
    try {
    cvtColor(image, gray, CV_BGR2GRAY);    
    }
    catch (cv::Exception e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
    int splitW = image.cols/SECTION_SIZE;
    int splitH = image.rows/SECTION_SIZE;
    double min, max;
    double adjusted_threshold;

    for(int i=0; i<splitH*splitW ; i++)
    {
        Mat part (gray, Rect(SECTION_SIZE*(i%splitW), SECTION_SIZE*(i/splitW), SECTION_SIZE, SECTION_SIZE) );

        cv::minMaxLoc(part, &min, &max);
        adjusted_threshold = max*light/255 + thresholdGray;
        //threshold(part, part, adjusted_threshold, 255, 1);
        inRange(part, 0, adjusted_threshold, part);

        Rect roi(SECTION_SIZE*(i%splitW), SECTION_SIZE*(i/splitW), SECTION_SIZE, SECTION_SIZE);
        part.copyTo( binaryIm(roi) );
    }
}

inline bool get_largest_contour(Mat &binaryIm, unsigned min_contour_area)
{
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    findContours( binaryIm, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    double largest_area=0;
    int contour_index=0;

    for( int i = 0; i< contours.size(); i++ )
    {
        double area = contourArea(contours[i]);
        if(area > largest_area)
        {
            largest_area = area;
            contour_index = i;
        }
    }
    binaryIm = Mat::zeros( binaryIm.size(), CV_8UC1 );

    bool big_area = (largest_area > min_contour_area);
    if(big_area) {
        drawContours( binaryIm, contours, contour_index, Scalar( 255, 255, 255 ), 2, 8, hierarchy, 0, Point() );
        // remove white edge at edge of image
        const int edge_remove_size = 5;
        binaryIm(Range::all(), Range(0,edge_remove_size)) = Scalar(0);
        binaryIm(Range::all(), Range(binaryIm.cols-edge_remove_size,binaryIm.cols) ) = Scalar(0);
        binaryIm(Range(0,edge_remove_size), Range::all()) = Scalar(0);
        binaryIm(Range(binaryIm.rows-edge_remove_size,binaryIm.rows), Range::all()) = Scalar(0);
    } // Else return black area
    return big_area;
}

inline bool advancedEdgeDetection(const Mat &image, Mat &edges)
{
    const unsigned thresholdGray = 70;
    const unsigned light = 70;
    edges = Mat::zeros( image.size(), CV_8UC1 ); //gray
    discrete_threshold(image, edges, thresholdGray, light);
    bool line_found = get_largest_contour(edges, MIN_COUNTOUR_AREA);
    return line_found;
}
