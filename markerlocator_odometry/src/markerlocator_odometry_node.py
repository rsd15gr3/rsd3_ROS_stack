#!/usr/bin/env python

import rospy
import socket
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


class MarkerLocatorOdometryNode:
    def __init__(self):
        # Params
        self.host = rospy.get_param("~host", '10.115.253.233')
        self.port = rospy.get_param("~port", 21212)
        self.frame_id = rospy.get_param("~frame_id", "marker_link")
        self.cov_diag_list = rospy.get_param("~pose_covariance_diagonal", [0.001, 0.001, 99999, 99999, 99999, 0.001])
        self.time_offset = rospy.get_param("~time_offset", 0.0)

        # Publishers
        self.odom_gps_pub = rospy.Publisher("odom_marloc", Odometry, queue_size=1)

        # Loop
        self.rate = rospy.Rate(2)
        self.loop()

    def get_marker7_position(self):
        # Socket connection to the MarkerLocator server
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # SOCK_STREAM is a TCP socket
        sock.connect((self.host, self.port))
        sent = sock.send("get position 7")

        if sent == 0:
            raise RuntimeError("Connection lost: Send failed")

        reply = sock.recv(16384)  # limit reply to 16K

        if reply == '':
            raise RuntimeError("Connection lost: Empty reply")

        sock.close()

        s = reply.strip().split(",")
        return {"order": s[0], "timestamp": float(s[1]), "x": float(s[2]), "y": float(s[3]), "angle": float(s[4])}

    def publish_odom_gps_message(self, marloc):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now() + rospy.Duration(self.time_offset)
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = marloc["x"]
        msg.pose.pose.position.y = marloc["y"]
        q = tf.transformations.quaternion_from_euler(0, 0, marloc["angle"])
        msg.pose.pose.orientation = Quaternion(*q)
        msg.pose.covariance = [self.cov_diag_list[0], 0, 0, 0, 0, 0,  # x
                               0, self.cov_diag_list[1], 0, 0, 0, 0,  # y
                               0, 0, self.cov_diag_list[2], 0, 0, 0,  # z
                               0, 0, 0, self.cov_diag_list[3], 0, 0,  # rot x
                               0, 0, 0, 0, self.cov_diag_list[4], 0,  # rot y
                               0, 0, 0, 0, 0, self.cov_diag_list[5]]  # rot z
        self.odom_gps_pub.publish(msg)

    def loop(self):
        while not rospy.is_shutdown():
            marloc = self.get_marker7_position()
            self.publish_odom_gps_message(marloc)
            self.rate.sleep()


if __name__ == '__main__':
    rospy.init_node("markerlocator_odometry_node")

    try:
        node = MarkerLocatorOdometryNode()
    except rospy.ROSInterruptException:
        pass
