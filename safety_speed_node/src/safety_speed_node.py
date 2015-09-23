#!/usr/bin/env python

"""
RSD :: Speed regulator node
input: linear_velocity, angular_velocity, speed_regulation
"""

import rospy
from geometry_msgs.msg import TwistStamped
from msgs.msg import FloatStamped


class Node():
    def __init__(self):
        self.update_rate = 20   # [Hz]

        # get topic names
        vel_topic = rospy.get_param('~cmd_vel_pub', '/fmCommand/cmd_vel')
        safe_vel_topic = rospy.get_param('~cmd_vel_pub', '/safe_vel')       # topic name TODO
        reg_ratio_topic = rospy.get_param('~cmd_vel_pub', '/reg_ratio')     # topic name TODO

        # setup variables
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.regulation_ratio = 1.0

        # setup subscription topics
        rospy.Subscriber(vel_topic, TwistStamped, self.on_vel_topic)
        rospy.Subscriber(reg_ratio_topic, FloatStamped, self.on_reg_ratio_topic)

        # setup publication topics
        self.safe_vel_msg = TwistStamped()
        self.safe_vel_pub = rospy.Publisher(safe_vel_topic, TwistStamped, queue_size=1)

        # frequency of node activity
        self.r = rospy.Rate(self.update_rate)
        self.updater()

    ''' Reads unregulated velocities '''
    def on_vel_topic(self, msg):
        self.vel_lin = msg.twist.linear.x
        self.vel_ang = msg.twist.angular.z

    ''' Reads the regulation ratio '''
    def on_reg_ratio_topic(self, msg):
        self.regulation_ratio = 1   # read here the regulation from <0,1> somehow TODO

    ''' Regulates velocities '''
    def regulate_velocities(self):
        self.safe_vel_msg.twist.linear.x = self.vel_lin*self.regulation_ratio
        self.safe_vel_msg.twist.angular.z = self.vel_ang*self.regulation_ratio

    ''' Regulates and publishes regulated velocities '''
    def publish_safe_vel_message(self):
        # The regulation is here
        self.regulate_velocities()

        # Publish
        self.safe_vel_msg.header.stamp = rospy.Time.now()
        self.safe_vel_pub.publish(self.safe_vel_msg)

    ''' Calls the publish function, based on the update_rate [Hz] '''
    def updater(self):
        while not rospy.is_shutdown():
            self.publish_safe_vel_message()
            self.r.sleep()


''' Main function, it all starts here '''
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('speed_regulator')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = Node()
    except rospy.ROSInterruptException:
        pass