#!/usr/bin/env python

import numpy as np
from skimage.measure import LineModel, ransac

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

def pol2cart(dist, angle):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return (x, y)

class WallExtractorNode():
    def __init__(self):
        # Topics
        laser_scan_topic = rospy.get_param("~laser_scan_sub", "/base_scan") # TODO default value
        marker_topic = rospy.get_param("~visualization_marker", "visualization_marker")

        # Publishers
        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1)

        # Subscribers
        self.scan = LaserScan()
        rospy.Subscriber(laser_scan_topic, LaserScan, self.on_laser_scan)

        # Loop
        self.rate = rospy.Rate(20)
        self.loop()

    def on_laser_scan(self, msg):
        self.scan = msg

    def init_marker(self):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "visualization_marker_frame"
        marker.ns = "wall_lines"
        marker.type = Marker.LINE_LIST
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        return marker

    def loop(self):
        while not rospy.is_shutdown():
            ranges = np.array(self.scan.ranges)
            angles = np.array([self.scan.angle_min + self.scan.angle_increment * i for i in range(len(ranges))])

            # Filter out ranges (and corresponding angles) not in the interval [range_min, range_max]
            good_indices = np.where(np.logical_and(ranges > self.scan.range_min, ranges < self.scan.range_max))
            ranges = ranges[good_indices]
            angles = angles[good_indices]

            # Skip iteration if too few good values
            if  np.size(good_indices) < 2:
                continue

            # Points in Cartesian coordinates
            points = [pol2cart(dist, angle) for (dist, angle) in zip(ranges, angles)]

            # Split points in the middle
            left = np.array(points[:len(points) / 2])
            right = np.array(points[len(points) / 2:])

            # Fit line with RANSAC algorithm
            left_model, inliers = ransac(left, LineModel, min_samples=2, residual_threshold=1, max_trials=1000)
            right_model, inliers = ransac(right, LineModel, min_samples=2, residual_threshold=1, max_trials=1000)

            # Predict y
            left_wall_x = np.array([left[0][0], left[-1][0]]) # two outermost points
            right_wall_x = np.array([right[0][0], right[-1][0]]) # two outermost points

            left_wall_y = left_model.predict_y(left_wall_x)
            right_wall_y = right_model.predict_y(right_wall_x)

            left_lines = []
            right_lines = []

            # Left line
            for p in zip(left_wall_x, left_wall_y):
                rp = Point()
                rp.x = p[0]
                rp.y = p[1]
                rp.z = 0.0
                left_lines.append(rp)

            # Right line
            for p in zip(right_wall_x, right_wall_y):
                rp = Point()
                rp.x = p[0]
                rp.y = p[1]
                rp.z = 0.0
                right_lines.append(rp)

            left_marker = self.init_marker()
            left_marker.color.g = 1.0
            left_marker.color.a = 1.0
            left_marker.points = left_lines

            right_marker = self.init_marker()
            right_marker.color.r = 1.0
            right_marker.color.a = 1.0
            right_marker.points = right_lines

            self.marker_pub.publish(left_marker)
            self.marker_pub.publish(right_marker)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_extractor')

    try:
        node = WallExtractorNode()
    except rospy.ROSInterruptException:
        pass
