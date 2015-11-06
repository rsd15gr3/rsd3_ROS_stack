/** QR decoder server node
*   This node subscribes to images from an USB camera and advertizes a service that returns a number indicating which of the known codes is seen currently.
*  If no known code is seen it returns -1.
**/

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <msgs/StringStamped.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>

using namespace std;
using namespace zbar;
using namespace cv;

#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG

sensor_msgs::Image rawImg;
const string node_name("zbar_decoder_node");
const string no_tag_id("?");
ImageScanner scanner;
string tag_value(no_tag_id);
bool prev_tag_found;
// change values to match workcells code
ros::Publisher pub;
vector<string> knownCodes;//

int debounce_size;
double scale_factor;
class BoolDebouncer
{
public:
  BoolDebouncer(unsigned size) : size(size), equals_count(0) {}
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

void camCallback(const sensor_msgs::Image::ConstPtr& img )
{
  // Convert image to opencv
  cv_bridge::CvImageConstPtr cv_ptr; // http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
  try
  {
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
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
    return;
  }
  bool current_tag_found = n > 0;
  bool tag_found = debouncer.update(current_tag_found);
  if(tag_found) // qr found?
  {
    if(current_tag_found)
    {
      ROS_DEBUG("Tag found");
      // first found tag is used
      tag_value = image.symbol_begin()->get_data();
    }
  }
  else if(prev_tag_found) // else n==0 -> no code found
  {
    ROS_DEBUG("Tag moved out of camera view");
    msgs::StringStamped msgs;
    msgs.header.stamp = ros::Time::now();
    msgs.header.frame_id = "camera_link";
    msgs.data = tag_value;
    pub.publish(msgs);
  }
  prev_tag_found = tag_found;
}

int main(int argc, char **argv){
  // Print debug msgs
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
#endif
  ros::init(argc, argv, node_name);
  ros::NodeHandle n("~");
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw",1, camCallback);
  pub = n.advertise<msgs::StringStamped>("/tag", 1);
  n.param<double>("tag_scale_factor",scale_factor, 0.5);
  n.param<int>("debounce_size", debounce_size, 3);
  if(debounce_size < 1) {
    debounce_size = 1;
    ROS_WARN("Debounce size less than 1 so defaults to: %i", debounce_size);
  }
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
  ros::spin();
}
