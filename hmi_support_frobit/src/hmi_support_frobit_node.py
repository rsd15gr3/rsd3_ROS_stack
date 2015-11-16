#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from msgs.msg import BoolStamped, IntStamped


class Node():

    def __init__(self):

        # Parameters
        self.vel_lin_max = rospy.get_param("~max_linear_velocity", 5)   # [m/s]
        self.vel_ang_max = rospy.get_param("~max_angular_velocity", 3)  # [rad/s]
        self.update_rate = rospy.Rate(20)                               # Hz

        # Topic names
        automode_topic = rospy.get_param("-automode_pub", "/fmPlan/automode")
        deadman_topic = rospy.get_param("~deadman_pub", "/fmSafe/deadman")
        cmd_vel_topic = rospy.get_param("~cmd_vel_pub", "/fmCommand/cmd_vel")
        ui_str_control_topic = rospy.get_param("~ui_str_control_sub", "/ui_str_control")

        # Setup automode publish topic
        self.automode_msg = IntStamped()
        self.automode_msg.data = 0
        self.automode_pub = rospy.Publisher(automode_topic, IntStamped, queue_size=1)

        # setup deadman publish topic
        self.deadman_msg = BoolStamped()
        self.deadman_msg.data = True
        self.deadman_pub = rospy.Publisher(deadman_topic, BoolStamped, queue_size=1)

        # setup manual velocity topic
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.js_x = None
        self.js_y = None
        self.js_x0 = None
        self.js_y0 = None
        self.x_diff_max = 100
        self.y_diff_max = 100
        self.cmd_vel_msg = TwistStamped()
        self.cmd_vel_pub = rospy.Publisher(cmd_vel_topic, TwistStamped, queue_size=1)

        # setup subscription topic callbacks
        rospy.Subscriber(ui_str_control_topic, String, self.on_ui_str_control_topic)

        # Updater function
        self.updater()

    def on_ui_str_control_topic(self, msg):
        if msg.data.startswith('mr_joystick'):
            data = msg.data.split(';')
            self.js_x0 = float(data[1])
            self.js_y0 = float(data[2])
            self.js_x = float(data[3])
            self.js_y = float(data[4])

            x_diff = abs(self.js_x-self.js_x0)
            y_diff = abs(self.js_y-self.js_y0)

            self.vel_ang = (x_diff/self.x_diff_max)*self.vel_ang_max
            self.vel_lin = (y_diff/self.y_diff_max)*self.vel_lin_max

            if self.js_x > self.js_x0:
                self.vel_ang *= -1

            if self.js_y > self.js_y0:
                self.vel_lin *= -1

        elif msg.data == 'mr_stop':
            self.vel_ang = 0.0
            self.vel_lin = 0.0
        elif msg.data == 'mr_mode_auto':
            self.automode_msg.data = 1
        elif msg.data == 'mr_mode_manual':
            self.automode_msg.data = 0

        # Limits
        self.vel_lin = min(self.vel_lin_max, max(self.vel_lin, -self.vel_lin_max))
        self.vel_ang = min(self.vel_ang_max, max(self.vel_ang, -self.vel_ang_max))

    def publish_automode_message(self):
        self.automode_msg.header.stamp = rospy.get_rostime()
        self.automode_pub.publish (self.automode_msg)

    def publish_deadman_message(self):
        self.deadman_msg.header.stamp = rospy.get_rostime()
        self.deadman_pub.publish (self.deadman_msg)

    def publish_cmd_vel_message(self):
        self.cmd_vel_msg.header.stamp = rospy.Time.now()
        self.cmd_vel_msg.twist.linear.x = self.vel_lin
        self.cmd_vel_msg.twist.angular.z = self.vel_ang
        self.cmd_vel_pub.publish(self.cmd_vel_msg)

    def updater(self):
        while not rospy.is_shutdown():
            self.publish_automode_message()
            self.publish_deadman_message()
            self.publish_cmd_vel_message()
            self.update_rate.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('hmi_support_frobit_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass


