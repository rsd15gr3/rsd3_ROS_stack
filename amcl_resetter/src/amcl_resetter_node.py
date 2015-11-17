#!/usr/bin/env python

import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class AMCLResetterNode:
    def __init__(self):
        rospy.init_node("amcl_resetter_node")

        self.dist_limit = 2
        self.odom_filtered = None
        self.odom_markerlocator = None

        self.tf_listener = tf.TransformListener()
        self.tf_listener.waitForTransform("map", "markerlocator", rospy.Time(0), rospy.Duration(5))

        rospy.Subscriber("odometry/filtered/local", Odometry, self.odom_filtered_cb)
        rospy.Subscriber("odometry/markerlocator", Odometry, self.odom_markerlocator_cb)

        self.init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=1)

        self.loop()

    def odom_filtered_cb(self, odom_msg):
        self.odom_filtered = odom_msg

    def odom_markerlocator_cb(self, odom_msg):
        self.odom_markerlocator = odom_msg
        self.calc_dist_and_publish()

    def odom_to_posestamped(self, odom_msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = odom_msg.header.stamp
        pose_msg.header.frame_id = odom_msg.header.frame_id
        pose_msg.pose = odom_msg.pose.pose
        return pose_msg

    def calc_dist_and_publish(self):
        if self.odom_filtered is None or self.odom_markerlocator is None:
            return

        # HACK
        t0 = rospy.Time(0)
        self.odom_filtered.header.stamp = t0
        self.odom_markerlocator.header.stamp = t0

        try:
            pose_filtered = self.tf_listener.transformPose("map", self.odom_to_posestamped(self.odom_filtered))
            pose_markerlocator = self.tf_listener.transformPose("map", self.odom_to_posestamped(self.odom_markerlocator))
        except (tf.LookupException, tf.ConnectivityException):
            return

        p1 = np.array((pose_filtered.pose.position.x, pose_filtered.pose.position.y))
        p2 = np.array((pose_markerlocator.pose.position.x, pose_markerlocator.pose.position.y))
        dist = np.linalg.norm(p2 - p1)

        if dist < self.dist_limit:
            initialpose_msg = PoseWithCovarianceStamped()
            initialpose_msg.header.stamp = rospy.Time.now()
            initialpose_msg.header.frame_id = "map"
            initialpose_msg.pose.pose = pose_markerlocator.pose
            self.init_pose_pub.publish(initialpose_msg)

    def loop(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = AMCLResetterNode()
    except rospy.ROSInterruptException:
        pass
