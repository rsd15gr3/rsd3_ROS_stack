/** QR decoder server node
*   This node subscribes to images from an USB camera and advertizes a service that returns a number indicating which of the known codes is seen currently.
*  If no known code is seen it returns -1.
**/

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <msgs/StringStamped.h>
#include <msgs/BoolStamped.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include "zbar_decoder/decode_qr.h"

using namespace std;
using namespace zbar;
using namespace cv;

#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG
const string node_name("zbar_decoder_node");

const double min_white_area = 100;
const unsigned min_tag_area = 70;
const int threshold_value = 210;
ImageScanner scanner;
bool prev_tag_found;
cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
ros::Publisher pub;
int debounce_size;
double scale_factor;
class BoolDebouncer
{
public:
  BoolDebouncer(unsigned size) : size(size), equals_count(0),
    prev_val(false), debuounced_val(false) {}
  bool update(bool val) {
    if(val == prev_val) {
      if(++equals_count > size) {
        equals_count = size; // to avoid overflow
        debuounced_val = val;
      }
    }
    else {
      equals_count = 0;
    }
    prev_val = val;
    return debuounced_val;
  }
private:
  unsigned size, equals_count;
  bool prev_val, debuounced_val;
};
BoolDebouncer debouncer(debounce_size);
bool decode_qr_tag(const cv_bridge::CvImageConstPtr &cv_ptr, string & value);
void camCallback(const sensor_msgs::Image::ConstPtr& img );
bool get_qr_id_callback(zbar_decoder::decode_qr::Response &req, zbar_decoder::decode_qr::Response &res);
unsigned getLargestContourROI(const Mat &im, Mat &mask, double min_area);
bool imHasQRTag(const Mat &im);

int main(int argc, char **argv){
  // Print debug msgs
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
#endif
  ros::init(argc, argv, node_name);
  ros::NodeHandle n("~");  
  prev_tag_found = false;
  pub = n.advertise<msgs::BoolStamped>("/tag_found", 1);
  n.param<double>("tag_scale_factor",scale_factor, 0.5);
  n.param<int>("debounce_size", debounce_size, 3);
  if(debounce_size < 1) {
    debounce_size = 1;
    ROS_WARN("Debounce size less than 1 so defaults to: %i", debounce_size);
  }
  else {
    ROS_INFO("Debounce size %i", debounce_size);
  }
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw",1, camCallback);
  ros::ServiceServer service = n.advertiseService("/get_qr_id", get_qr_id_callback);
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

  ros::spin();
}

bool imHasQRTag(const Mat &im)
{
  Mat binary_im;
  threshold( im, binary_im, threshold_value, 255, CV_THRESH_BINARY );
  Mat mask;
  unsigned roi_size = getLargestContourROI(binary_im, mask, min_white_area);
  if(roi_size > min_white_area)
  {
    Mat roi;
    bitwise_not(binary_im, roi, mask);
    unsigned tag_area = countNonZero(roi);
    if(tag_area > min_tag_area)
    {
      return true;
    }
  }
  return false;
}

bool get_qr_id_callback(zbar_decoder::decode_qr::Response &req, zbar_decoder::decode_qr::Response &res)
{
  string qr_tag;
  bool tag_in_im = decode_qr_tag(cv_ptr, qr_tag);
  res.value = qr_tag;
}

bool decode_qr_tag(const cv_bridge::CvImageConstPtr &cv_ptr, string & value)
{
  // extreact qr code
  cv::Mat down_scaled_im;
  cv::resize(cv_ptr->image, down_scaled_im, Size(), scale_factor, scale_factor);
  uchar *raw = (uchar *)down_scaled_im.data;
  unsigned width = down_scaled_im.cols;
  unsigned height = down_scaled_im.rows;
  // wrap image data
  Image image(width, height, "Y800", raw, width * height);
  // scan the image for barcodes
  int n = scanner.scan(image);
  if(n < 0)
  {
    ROS_ERROR_NAMED(node_name,"Error decoding image with Image scan");
  }
  else
  {
    value = image.symbol_begin()->get_data();
  }
  return n > 0;
}

unsigned getLargestContourROI(const Mat &im, Mat &mask, double min_area)
{
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  Mat tmp = im.clone();
  findContours( tmp, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  double largest_contour_area = min_area;
  unsigned maxAreaIdx = -1;
  vector<Point> largest_contour;
  for( int i = 0; i< contours.size(); i++ )
  {
    double area = contourArea(contours[i]);
    if(area > largest_contour_area) {
      maxAreaIdx = i;
      largest_contour_area=area;
    }
  }
  if(maxAreaIdx == -1)
  {
    //ROS_DEBUG("No barcode");
  }
  else
  {
    mask = cv::Mat::zeros(im.size(), im.type());
    cv::drawContours(mask, contours, maxAreaIdx, cv::Scalar(255),CV_FILLED);
  }
  return largest_contour_area;
}

void camCallback(const sensor_msgs::Image::ConstPtr& img )
{
  // Convert image to opencv
  try
  {
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  bool current_tag_found = imHasQRTag(cv_ptr->image);
  bool tag_found = debouncer.update(current_tag_found);
  //if(tag_found)
    //ROS_INFO("Tag found");
  //else
    //ROS_INFO("no tag found");
  if(tag_found != prev_tag_found) // qr found?
  {
    //ROS_DEBUG("Tag found");
    // first found tag is used
    //ROS_DEBUG_COND(qr_tag_found, "qr tag not found by zbar");
    msgs::BoolStamped msgs;
    msgs.header.stamp = ros::Time::now();
    msgs.header.frame_id = "camera_link";
    msgs.data = tag_found;
    pub.publish(msgs);
  }
  prev_tag_found = tag_found;
}
