#!/usr/bin/env python

"""
RSD :: Speed regulator node
input: linear_velocity, angular_velocity, speed_regulation
"""

import rospy
from geometry_msgs.msg import TwistStamped
from msgs.msg import FloatArrayStamped, BoolStamped


class Node():
    def __init__(self):
        self.update_rate = 20   # [Hz]

        # get topic names
        vel_topic = rospy.get_param('~vel_topic', '/fmCommand/cmd_vel')
        safe_vel_topic = rospy.get_param('~safe_vel_topic', '/fmCommand/safe_vel')       # topic name TODO
        reg_ratio_topic = rospy.get_param('~reg_ratio_topic', '/reg_ratio')     # topic name TODO
        disable_topic = rospy.get_param('~disable_topic', '/disable_safety_speed')

        # setup variables
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.regulation_ratio = [0, 0]
        self.safetyOn = True
        self.disable = True

        # setup subscription topics
        rospy.Subscriber(vel_topic, TwistStamped, self.on_vel_topic)
        rospy.Subscriber(reg_ratio_topic, FloatArrayStamped, self.on_reg_ratio_topic)
        rospy.Subscriber(disable_topic, BoolStamped, self.disableCB)

        # setup publication topics
        self.safe_vel_msg = TwistStamped()
        self.safe_vel_pub = rospy.Publisher(safe_vel_topic, TwistStamped, queue_size=1)

        # frequency of node activity
        self.r = rospy.Rate(self.update_rate)
        self.updater()

    def disableCB (self, msg):
        self.disable = msg.data

    ''' Reads unregulated velocities '''
    def on_vel_topic(self, msg):
        self.vel_lin = msg.twist.linear.x
        self.vel_ang = msg.twist.angular.z

    ''' Reads the regulation ratio '''
    def on_reg_ratio_topic(self, msg):
        self.regulation_ratio = msg.data   # read here the regulation from <0,1> somehow TODO

    ''' Regulates velocities '''
    def regulate_velocities(self):
        self.safe_vel_msg.twist.linear.x = self.vel_lin*self.regulation_ratio[0]
        self.safe_vel_msg.twist.angular.z = self.vel_ang*self.regulation_ratio[0] # reg_ration[1] not used

    ''' Regulates and publishes regulated velocities '''
    def publish_safe_vel_message(self):
        # The regulation is here
        if self.disable == False:
            self.regulate_velocities()
        else:
            self.safe_vel_msg.twist.linear.x = self.vel_lin
            self.safe_vel_msg.twist.angular.z = self.vel_ang

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
    rospy.init_node('safety_speed_node')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = Node()
    except rospy.ROSInterruptException:
        pass
