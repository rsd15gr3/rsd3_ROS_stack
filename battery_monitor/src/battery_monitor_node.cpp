#include <ros/ros.h>
#include <msgs/nmea.h>
#include <msgs/FloatStamped.h>
#include <msgs/BoolStamped.h>
#define ROSCONSOLE_MIN_SEVERITY ROSCONSOLE_SEVERITY_DEBUG //ROSCONSOLE_SEVERITY_INFO, ROSCONSOLE_SEVERITY_DEBUG
using namespace std;
using namespace ros;

double frobit_voltage = 0;
double robocard_voltage_divider = 0.03747; // 5.0*(22000 + 3300)/(3300*1023) because of 22000/3300 ohm v-divider

void nmeaFromFrobitCb(msgs::nmeaConstPtr msg)
{
  if (msg->valid == true && msg->type == "PFBST")
  {
    frobit_voltage = robocard_voltage_divider * atoi(msg->data[3].c_str());
  }
}

int main(int argc, char **argv)
{
#if ROSCONSOLE_MIN_SEVERITY == ROSCONSOLE_SEVERITY_DEBUG
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }
#endif
  init(argc, argv, "battery_monitor_node");
  NodeHandle nh("~");
  double min_battery_level;
  int publish_rate;
  nh.param<double>("min_battery_level", min_battery_level, 12.0);
  nh.param<int>("publish_rate", publish_rate, 5);
  Subscriber nmea_sub;
  {
    string nmea_to_frobit_sub;
    nh.param<string>("nmea_from_frobit_sub", nmea_to_frobit_sub, "/fmSignal/nmea_from_frobit");
    nmea_sub = nh.subscribe<msgs::nmeaConstPtr>(nmea_to_frobit_sub, 1,&nmeaFromFrobitCb);
  }
  Publisher voltage_pub = nh.advertise<msgs::FloatStamped>("battery_level", 1);
  Publisher too_low_battery_pub = nh.advertise<msgs::BoolStamped>("too_low_battery", 1);
  Rate r(publish_rate);
  while (nh.ok()) {
    ros::spinOnce();
    r.sleep();
    msgs::FloatStamped battery_level;
    battery_level.header.stamp = Time::now();
    battery_level.data = frobit_voltage;
    ROS_DEBUG("voltage= %f", frobit_voltage);
    voltage_pub.publish(battery_level);
    msgs::BoolStamped too_low_battery_msgs;
    too_low_battery_msgs.header.stamp = Time::now();
    too_low_battery_msgs.data = frobit_voltage < min_battery_level;
    ROS_DEBUG("below minimum = %i", frobit_voltage < min_battery_level);
    too_low_battery_pub.publish(too_low_battery_msgs);
  }
  return 0;
}
