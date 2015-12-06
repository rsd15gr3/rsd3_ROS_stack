#!/usr/bin/env python

"""
ActionServer Node for docking Frobit to the charger

action server name: docking_with_walls_server

subscribing:
- /fmPlan/automode
- /fmSensors/scan

publishing:
- /fmCommand/cmd_vel    (20Hz if automode=1, Control for docking, TwistStamped)

"""

import rospy
from msgs.msg import IntStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from docking_with_walls.msg import docking_with_wallsAction
import numpy as np
import actionlib


class DockingActionNode():

    def __init__(self):
        """ Node for Docking ActionServer Instance Initialization """

        ''' Topics '''
        self.tp_scan = '/fmSensors/scan'    # Real Cruel World
        #self.tp_scan = '/base_scan'        # Sim
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
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.left_wall = 0.0
        self.front_left_wall = 0
        self.front_right_wall = 0

        ''' Subscribers '''
        rospy.Subscriber(self.tp_scan, LaserScan, self.on_tp_scan)
        rospy.Subscriber(self.tp_frobit_automode, IntStamped, self.on_tp_frobit_automode)

        ''' ActionServer '''
        self.finished = False
        self.success = True
        self.action_name = 'docking_with_walls_server'
        self.action_server = actionlib.SimpleActionServer(self.action_name, docking_with_wallsAction, execute_cb=self.run_action, auto_start=False)
        self.action_server.start()

    def on_tp_scan(self, msg):
        del self.scan_ranges[:]
        for i, a_range in enumerate(msg.ranges):
            if msg.range_min < a_range < msg.range_max:
                self.scan_ranges.append(a_range)

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
                self.left_wall = np.mean(self.scan_ranges[len(self.scan_ranges)-20:len(self.scan_ranges)-10])
                self.front_left_wall = np.mean(self.scan_ranges[len(self.scan_ranges)/2:len(self.scan_ranges)/2+10])
                self.front_right_wall = np.mean(self.scan_ranges[len(self.scan_ranges)/2-10:len(self.scan_ranges)/2])
            except IndexError:
                pass
        
        if abs(self.front_left_wall-0.08) < 0.02 and abs(self.front_right_wall-0.08) < 0.02:
            self.action_server.set_succeeded()
            self.finished = True
            self.stop_frobit()
            print 'Mission completed.'
        else:
            self.vel_lin = 0.1

            if self.left_wall > 0.35:
                self.vel_ang = 0.0
            else:
                if self.left_wall < 0.295:
                    self.vel_ang = -0.15
                elif self.left_wall > 0.299:
                    self.vel_ang = 0.15
        self.publish_tp_cmd_vel_message()

    def stop_frobit(self):
        self.vel_ang = 0.0
        self.vel_lin = 0.0

    def run_action(self, _goal_):
        while not self.finished and self.success and not rospy.is_shutdown():
            if self.action_server.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self.action_name)
                self.action_server.set_preempted()
                self.success = False
                break
            if self.frobit_automode:
                self.go_to_goal()
            self.publishing_rate.sleep()

if __name__ == '__main__':
    """ The program starts here by naming the node and initializing it. """

    rospy.init_node('docking_with_walls_node')
    DockingActionNode()
    rospy.spin()
