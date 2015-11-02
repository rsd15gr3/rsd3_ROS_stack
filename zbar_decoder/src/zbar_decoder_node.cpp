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
#include <zbar.h>

using namespace std;
using namespace zbar;
using namespace cv;

#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_ERROR //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG

sensor_msgs::Image rawImg;
const string node_name("zbar_decoder_node");
ImageScanner scanner;
string previous_tag_value;
// change values to match workcells code
vector<string> knownCodes;// {"http://www.youtube.com/watch?v=S5KafuOcyn4", "http://www.qrstuff.com", "IT'S BACK ON DVD SEPTEMBER 10th" };
ros::Publisher pub;

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
  uchar *raw = (uchar *)cv_ptr->image.data;
  unsigned width = cv_ptr->image.cols;
  unsigned height = cv_ptr->image.rows;
  // wrap image data
  Image image(width, height, "Y800", raw, width * height);
  // scan the image for barcodes
  int n = scanner.scan(image);
  if(n > 0) // qr found?
  {
    string tagValue("Unknown");
    bool knownCodeNotFound = true;
    // find first known tag
    for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end() && knownCodeNotFound; ++symbol)
    {
      for(int i=0; i<knownCodes.size(); i++)
      {
        if(symbol->get_data() == knownCodes[i])
        {
          tagValue = symbol->get_data();
          knownCodeNotFound = false;
          break;
        }
      }
    }
    if(knownCodeNotFound)
    {
      ROS_DEBUG_NAMED(node_name,"No known code found");
    }
    else
    {
      ROS_DEBUG(tagValue.c_str());
      if(tagValue != previous_tag_value)
      {
        previous_tag_value = tagValue;
        msgs::StringStamped msgs;
        msgs.header.stamp = ros::Time::now();
        msgs.header.frame_id = "camera_link";
        msgs.data = tagValue;
        pub.publish(msgs);
      }
    }
  }
  else if(n < 0)
  {
    ROS_ERROR_NAMED(node_name,"Error decoding image with Image scan");
    return;
  }
  else // else n==0 -> no known kode found
  {
    ROS_DEBUG_NAMED(node_name,"No qr code found");

  }
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
  n.param<vector<string> >("known_qr_codes", knownCodes, vector<string>());
  ros::Subscriber sub = n.subscribe("/usb_cam/image_raw",1, camCallback);
  pub = n.advertise<msgs::StringStamped>("/tag", 1);
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
  ros::spin();
}
