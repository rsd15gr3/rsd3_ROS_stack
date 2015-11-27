#!/usr/bin/env python

"""
RSD :: Obstacle detection node
input: laser scan
"""

import rospy
from msgs.msg import FloatArrayStamped
from sensor_msgs.msg import LaserScan
from math import *


class mission_node():

    def __init__(self):
        self.update_rate = 20  # [Hz]

        reg_ratio_topic = rospy.get_param("~reg_ratio_topic", "/reg_ratio")
        scan_topic = rospy.get_param("~scan_topic", "/fmSensors/scan")

        # setup obstacle publish topic
        self.reg = [0, 0]
        self.reg_msg = FloatArrayStamped()
        self.reg_pub = rospy.Publisher(reg_ratio_topic, FloatArrayStamped, queue_size=1)

        # setup obstacle publish topic
        #self.obstacle = true
        #self.obstacle_msg = BoolStamped()
        #self.obstacle_pub = rospy.Publisher(obstacle_topic, BoolStamped, queue_size=1)

        # setup subscription topic callbacks
        rospy.Subscriber(scan_topic, LaserScan, self.on_scan_topic)

        # sall updater function
        self.r = rospy.Rate(self.update_rate)
        self.updater()

    # def on_scan_topic(self, msg):
    #   size = len(msg.ranges)
    #   counter = 0
    #   for x in range(0, 10):
    #       if msg.ranges[(size/2)+x]<0.5 or msg.ranges[(size/2)-x]<0.5:
    #           counter = counter +1

    #   if counter>4:
    #       self.obstacle=True
    #   else:
    #       self.obstacle=False

    def on_scan_topic(self, msg):
        size = len(msg.ranges)
        ranges = []
        angles = []
        for i in range(0, size):
            angle = msg.angle_min + msg.angle_increment * i
            if msg.ranges[i] > msg.range_min and msg.ranges[i] < msg.range_max:
                ranges.append(msg.ranges[i])
                angles.append(angle)

        # for n in range(0, size):
        #   if ranges[n] > self.msg.range_max or ranges[n] < self.msg.range_min:

        min = 1
        width = rospy.get_param("diff_steer_wheel_distance", 0.23) / 2  # half the robots width

        for n in range(0, len(ranges)):
            x = sin(angles[n]) * ranges[n]
            y = cos(angles[n]) * ranges[n]

            if x < width and x > -width:
                if y < min and y > 0.1:
                    min = y

        if min < 0.3:
            self.reg[0] = 0
        elif min > 0.99:  # min == 1
            self.reg[0] = 1
        else:
            self.reg[0] = (min - 0.3) / 0.7  # normalize

        if min < 0.15:
            self.reg[1] = 0
        elif min > 0.99:
            self.reg[1] = 1
        else:
            self.reg[1] = (min - 0.15) / 0.7

    def publish_speed_message(self):
        self.reg_msg.data = self.reg
        self.reg_msg.header.stamp = rospy.get_rostime()
        self.reg_pub.publish(self.reg_msg)

    # def publish_obstacle_message(self):
    #   self.obstacle_msg.data = self.obstacle
    #   self.obstacle_msg.header.stamp = rospy.get_rostime()
    #   self.obstacle_pub.publish(self.obstacle_msg)

    def updater(self):
        while not rospy.is_shutdown():
            self.publish_speed_message()

            self.r.sleep()

# main function.
if __name__ == '__main__':
    # initialize the node and name it.
    rospy.init_node('lidar_obstacle_node')

    # go to class functions that do all the heavy lifting. Do error checking.
    try:
        node_class = mission_node()
    except rospy.ROSInterruptException:
        pass
