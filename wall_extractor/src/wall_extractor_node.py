#!/usr/bin/env python

import itertools
import numpy as np
import rospy
from geometry_msgs.msg import Point
from msgs.msg import row  # Frobomind 'row' message
from sensor_msgs.msg import LaserScan
from skimage.measure import LineModel, ransac
from visualization_msgs.msg import Marker


def pol2cart(dist, angle):
    # Angle is in radians.
    x = dist * np.cos(angle)
    y = dist * np.sin(angle)
    return (x, y)


class WallExtractorNode():
    def __init__(self):
        # Topics
        laser_scan_topic = rospy.get_param("~laser_scan_sub", "/scan")
        marker_topic = rospy.get_param("~ransac_debug_marker", "ransac_debug_marker")
        walls_topic = rospy.get_param("~walls", "walls")

        # Publishers
        self.marker_pub = rospy.Publisher(marker_topic, Marker, queue_size=1)
        self.rows_pub = rospy.Publisher(walls_topic, row, queue_size=1)

        # Subscribers
        self.scan = LaserScan()
        rospy.Subscriber(laser_scan_topic, LaserScan, self.on_laser_scan)

        # Loop
        self.rate = rospy.Rate(10)
        self.loop()

    def on_laser_scan(self, msg):
        self.scan = msg

    def publish_wall(self, left_model, left_valid, right_model, right_valid):
        # skimage LineModel gives the angle to the line perpendicular to the
        # actual line, so we rotate it 90 deg.
        # Refer to Anders for documentation on error_ values
        row_msg = row()
        row_msg.header.stamp = rospy.Time.now()
        row_msg.header.frame_id = "laser"
        row_msg.rightvalid = right_valid
        row_msg.rightdistance = right_model.params[0]
        row_msg.rightangle = right_model.params[1] - np.pi / 2
        row_msg.rightvar = 0
        row_msg.leftvalid = left_valid
        row_msg.leftdistance = left_model.params[0]
        row_msg.leftangle = left_model.params[1] - np.pi / 2
        row_msg.leftvar = 0
        row_msg.headland = True if not left_valid or not right_valid else False
        row_msg.error_angle = (left_model.params[1] + right_model.params[1]) / 2
        row_msg.error_distance = (np.absolute(left_model.params[0]) - np.absolute(right_model.params[0])) / 2
        row_msg.var = 0
        self.rows_pub.publish(row_msg)

    def format_viz_points(self, x_list, y_list):
        if len(x_list) != len(y_list):
            raise ValueError("x_list and y_list must be of same length")

        points = []

        for x, y in itertools.izip(x_list, y_list):
            point_msg = Point()
            point_msg.x = x
            point_msg.y = y
            point_msg.z = 0.0
            points.append(point_msg)

        return points

    def publish_visualization_marker(self, x, y, mtype, ns, rgb):
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "laser"
        marker_msg.type = mtype
        marker_msg.action = marker_msg.ADD
        marker_msg.ns = ns
        marker_msg.pose.orientation.w = 1.0
        marker_msg.scale.x = 0.03
        marker_msg.color.a = 0.9
        marker_msg.color.r = rgb[0]
        marker_msg.color.g = rgb[1]
        marker_msg.color.b = rgb[2]
        marker_msg.points = self.format_viz_points(x, y)
        self.marker_pub.publish(marker_msg)

    def loop(self):
        while not rospy.is_shutdown():
            ranges = np.array(self.scan.ranges)
            angles = np.array([self.scan.angle_min + self.scan.angle_increment * i for i in range(len(ranges))])

            # Filter out ranges (and corresponding angles) not in the interval [range_min, range_max].
            good_indices = np.where(np.logical_and(ranges > self.scan.range_min, ranges < self.scan.range_max))
            ranges = ranges[good_indices]
            angles = angles[good_indices]

            # Skip iteration if too few good values.
            if np.size(good_indices) < 2:
                continue

            # Points in Cartesian coordinates.
            points = np.array([pol2cart(dist, angle) for dist, angle in itertools.izip(ranges, angles)])

            # Split points in the middle.
            left_points = points[:len(points) / 2]
            right_points = points[len(points) / 2:]

            # Fit line models with RANSAC algorithm.
            left_model, left_inliers = ransac(left_points, LineModel, min_samples=5, residual_threshold=0.1, max_trials=100)
            right_model, right_inliers = ransac(right_points, LineModel, min_samples=5, residual_threshold=0.1, max_trials=100)

            # Determine validity of the lines
            left_valid = True
            right_valid = True

            if np.size(left_inliers) < 15:
                left_valid = False

            if np.size(right_inliers) < 15:
                right_valid = False

            # Publish row message.
            self.publish_wall(left_model, left_valid, right_model, right_valid)

            # RViz visualization of lines and which points are considered in/outliers.
            # Predict y's using the two outermost x's. This gives us two points on each line.
            left_wall_x = np.array([left_points[0][0], left_points[-1][0]])
            right_wall_x = np.array([right_points[0][0], right_points[-1][0]])
            left_wall_y = left_model.predict_y(left_wall_x)
            right_wall_y = right_model.predict_y(right_wall_x)

            # Publish markers.
            self.publish_visualization_marker(left_wall_x, left_wall_y, Marker.LINE_STRIP, "line_left", (0.2, 1.0, 0.2))
            self.publish_visualization_marker(right_wall_x, right_wall_y, Marker.LINE_STRIP, "line_right", (0.2, 0.2, 1.0))
            self.publish_visualization_marker(left_points[left_inliers, 0], left_points[left_inliers, 1], Marker.POINTS, "left_inliers", (0.5, 1.0, 0.5))
            self.publish_visualization_marker(right_points[right_inliers, 0], right_points[right_inliers, 1], Marker.POINTS, "right_inliers", (0.5, 0.5, 1.0))
            left_outliers = left_inliers == False
            right_outliers = right_inliers == False
            self.publish_visualization_marker(left_points[left_outliers, 0], left_points[left_outliers, 1], Marker.POINTS, "left_outliers", (0.0, 0.5, 0.0))
            self.publish_visualization_marker(right_points[right_outliers, 0], right_points[right_outliers, 1], Marker.POINTS, "right_outliers", (0.0, 0.0, 0.5))

            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node('wall_extractor')

    try:
        node = WallExtractorNode()
    except rospy.ROSInterruptException:
        pass
