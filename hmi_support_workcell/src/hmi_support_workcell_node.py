#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from msgs.msg import BoolStamped, IntStamped


class Node():

    def __init__(self):

        # Parameters
        self.update_rate = rospy.Rate(20)								# Hz

        # Topic names
        ui_str_control_wc_topic = rospy.get_param("~ui_str_control_wc_sub", "/ui_str_control_wc")

        # setup subscription topic callbacks
        rospy.Subscriber(ui_str_control_wc_topic, String, self.on_ui_str_control_wc_topic)

        # Updater function
        self.updater()

    def on_ui_str_control_wc_topic(self, msg):
        pass

    def updater(self):
        while not rospy.is_shutdown():
            self.update_rate.sleep()

# main function.    
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('hmi_support_workcell_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass


