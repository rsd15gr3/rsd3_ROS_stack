#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import Point
from msgs.msg import row  # Frobomind 'row' message
from sensor_msgs.msg import LaserScan
from skimage.measure import LineModel, ransac
from visualization_msgs.msg import Marker


def pol2cart(dist, angle):
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return (x, y)


class WallExtractorNode():
    def __init__(self):
        # Topics
        laser_scan_topic = rospy.get_param("~laser_scan_sub", "/scan")  # TODO default value
        marker_topic = rospy.get_param("~visualization_marker", "visualization_marker")
        rows_topic = rospy.get_param("~rows", "rows")

        # Publishers
        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1)
        self.rows_pub = rospy.Publisher(rows_topic, row, queue_size=1)

        # Subscribers
        self.scan = LaserScan()
        rospy.Subscriber(laser_scan_topic, LaserScan, self.on_laser_scan)

        # Loop
        self.rate = rospy.Rate(10)
        self.loop()

    def on_laser_scan(self, msg):
        self.scan = msg

    def format_viz_points(self, x, y):
        vis_points = []

        for p in zip(x, y):
            rp = Point()
            rp.x = p[0]
            rp.y = p[1]
            rp.z = 0.0
            vis_points.append(rp)

        return vis_points

    def publish_visualization_marker(self, x, y, mtype, ns, rgb):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "laser"
        marker.type = mtype
        marker.action = Marker.ADD
        marker.ns = ns
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.a = 0.9
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.points = self.format_viz_points(x, y)
        self.marker_pub.publish(marker)

    def publish_row(self, left_model, right_model):
        rowmsg = row()
        rowmsg.header.stamp = rospy.Time.now()
        rowmsg.header.frame_id = "laser"

        rowmsg.rightvalid = 1 # TODO
        rowmsg.rightdistance = right_model.params[0]
        rowmsg.rightangle = right_model.params[1]
        rowmsg.rightvar = 0

        rowmsg.leftvalid = 1 # TODO
        rowmsg.leftdistance = left_model.params[0]
        rowmsg.leftangle = left_model.params[1]
        rowmsg.leftvar = 0

        rowmsg.headland = 0
        rowmsg.error_angle = 0
        rowmsg.error_distance = 0
        rowmsg.var = 0

        self.rows_pub.publish(rowmsg)


    def loop(self):
        while not rospy.is_shutdown():
            ranges = np.array(self.scan.ranges)
            angles = np.array([self.scan.angle_min + self.scan.angle_increment * i for i in range(len(ranges))])

            # Filter out ranges (and corresponding angles) not in the interval [range_min, range_max]
            good_indices = np.where(np.logical_and(ranges > self.scan.range_min, ranges < self.scan.range_max))
            ranges = ranges[good_indices]
            angles = angles[good_indices]

            # Skip iteration if too few good values
            if np.size(good_indices) < 2:
                continue

            # Points in Cartesian coordinates
            points = np.array([pol2cart(dist, angle) for (dist, angle) in zip(ranges, angles)])

            # Split points in the middle. HACK Manually excluding some points in the middle
            left = points[:len(points) / 2 - 3]
            right = points[len(points) / 2 + 3:]

            # Fit lines with RANSAC algorithm
            left_model, left_inliers = ransac(left, LineModel, min_samples=3, residual_threshold=0.1, max_trials=100)
            right_model, right_inliers = ransac(right, LineModel, min_samples=3, residual_threshold=0.1, max_trials=100)

            self.publish_row(left_model, right_model)

            # Predict y's using the two outermost points. This gives us two points on each line.
            left_wall_x = np.array([left[0][0], left[-1][0]])
            right_wall_x = np.array([right[0][0], right[-1][0]])
            left_wall_y = left_model.predict_y(left_wall_x)
            right_wall_y = right_model.predict_y(right_wall_x)

            # RViz visualization of lines and which points are considered in/outliers
            self.publish_visualization_marker(left_wall_x, left_wall_y, Marker.LINE_STRIP, "line_left", (0.2, 1.0, 0.2))
            self.publish_visualization_marker(right_wall_x, right_wall_y, Marker.LINE_STRIP, "line_right", (0.2, 0.2, 1.0))
            self.publish_visualization_marker(left[left_inliers, 0], left[left_inliers, 1], Marker.POINTS, "left_inliers", (0.5, 1.0, 0.5))
            self.publish_visualization_marker(right[right_inliers, 0], right[right_inliers, 1], Marker.POINTS, "right_inliers", (0.5, 0.5, 1.0))
            left_outliers = left_inliers == False
            right_outliers = right_inliers == False
            self.publish_visualization_marker(left[left_outliers, 0], left[left_outliers, 1], Marker.POINTS, "left_outliers", (0.0, 0.5, 0.0))
            self.publish_visualization_marker(right[right_outliers, 0], right[right_outliers, 1], Marker.POINTS, "right_outliers", (0.0, 0.0, 0.5))

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('wall_extractor')

    try:
        node = WallExtractorNode()
    except rospy.ROSInterruptException:
        pass
