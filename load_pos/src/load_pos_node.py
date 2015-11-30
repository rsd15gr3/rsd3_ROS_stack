#!/usr/bin/env python

import rospy
from msgs.msg import IntStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from math import sin, cos
import numpy as np


class Node():

    def __init__(self):
        """ Node Instance Initialization """

        ''' Topics '''
        self.tp_scan = '/fmSensors/scan'
        self.tp_frobit_automode = '/fmPlan/automode'
        self.tp_cmd_vel = '/fmCommand/cmd_vel'

        ''' Publishers '''
        self.tp_cmd_vel_message = TwistStamped()
        self.tp_cmd_vel_publisher = rospy.Publisher(self.tp_cmd_vel, TwistStamped, queue_size=1)

        ''' Publishing rate '''
        self.publishing_rate = rospy.Rate(10)          # [Hz]

        ''' Variables '''
        self.frobit_automode = 0
        self.scan_ranges = list()
        self.scan_angles = list()
        self.c_x = list()
        self.c_y = list()
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.right_wall = 40
        self.front_left_wall = 0
        self.front_right_wall = 0

        ''' Subscribers '''
        rospy.Subscriber(self.tp_scan, LaserScan, self.on_tp_scan)
        rospy.Subscriber(self.tp_frobit_automode, IntStamped, self.on_tp_frobit_automode)

        # Loop function
        self.keep_publishing()

    def on_tp_scan(self, msg):
        del self.scan_ranges[:]
        del self.scan_angles[:]
        del self.c_x[:]
        del self.c_y[:]
        for i, a_range in enumerate(msg.ranges):
            if msg.range_min < a_range < msg.range_max:
                angle = msg.angle_min + msg.angle_increment * i
                self.scan_ranges.append(a_range)
                self.scan_angles.append(angle)
                self.c_x.append(cos(angle)*a_range)
                self.c_y.append(sin(angle)*a_range)

    def on_tp_frobit_automode(self, msg):
        self.frobit_automode = msg.data

    def publish_tp_cmd_vel_message(self):
        self.tp_cmd_vel_message.header.stamp = rospy.Time.now()
        self.tp_cmd_vel_message.twist.linear.x = self.vel_lin
        self.tp_cmd_vel_message.twist.angular.z = self.vel_ang
        self.tp_cmd_vel_publisher.publish(self.tp_cmd_vel_message)

    def go_to_goal(self):
        if self.scan_ranges:
            try:
                self.right_wall = np.mean(self.scan_ranges[30:80])
                self.front_left_wall = np.mean(self.scan_ranges[len(self.scan_ranges)/2+10:len(self.scan_ranges)/2+30])
                self.front_right_wall = np.mean(self.scan_ranges[len(self.scan_ranges)/2-30:len(self.scan_ranges)/2-10])
            except IndexError:
                pass

        if 0.4 < self.front_left_wall < 0.6 and 0.4 < self.front_right_wall < 0.6:
            print 'front wall seen', self.front_left_wall, self.front_right_wall
            self.vel_lin = 0.0
            self.vel_ang = 0.0
            if abs(self.front_right_wall-self.front_left_wall) > 0.01:
                if self.front_right_wall > self.front_left_wall:
                    self.vel_ang = 0.1
                else:
                    self.vel_ang = -0.1
        else:
            self.vel_lin = 0.15

            if self.right_wall < 0.40:
                self.vel_ang = 0.35
                print 'right wall close, turning left', self.right_wall
            elif self.right_wall > 0.45:
                self.vel_ang = -0.2
                print 'right wall far away, turning right', self.right_wall
            else:
                self.vel_ang = 0.0
        #print 'front distance:', self.front_left_wall, self.front_right_wall

        self.publish_tp_cmd_vel_message()

    def stop_frobit(self):
        self.vel_ang = 0.0
        self.vel_lin = 0.0

    def keep_publishing(self):
        while not rospy.is_shutdown():
            if self.frobit_automode:
                self.go_to_goal()
            self.publishing_rate.sleep()

if __name__ == '__main__':
    """ The program starts here by naming the node and initializing it. """

    rospy.init_node('load_pos_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass
