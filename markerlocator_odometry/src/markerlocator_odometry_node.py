#!/usr/bin/env python

import rospy
import socket
from nav_msgs.msg import Odometry


class MarkerLocatorOdometryNode:
    def __init__(self):
        # Params
        host = rospy.get_param("~host", '10.115.253.233')
        port = rospy.get_param("~port", 21212)
        self.frame_id = rospy.get_param("~frame_id", "base_link")
        self.cov_x = rospy.get_param("~covariance_x", 0.01)
        self.cov_y = rospy.get_param("~covariance_y", 0.01)
        self.cov_z = rospy.get_param("~covariance_z", 0.01)
        self.cov_rot = rospy.get_param("~covariance_rot", 99999)

        # Publishers
        self.gps_pub = rospy.Publisher("odom_gps", Odometry, queue_size=1)

        # Socket connection to the MarkerLocator server
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # SOCK_STREAM is a TCP socket
        self.sock.connect((host, port))

        # Loop
        self.rate = rospy.Rate(30)
        self.loop()

    def get_marker5_position(self):
        sent = self.sock.send("Get position 5")

        if sent == 0:
            raise RuntimeError("Connection broken")

        reply = self.sock.recv(16384)  # limit reply to 16K

        if reply == '':
            raise RuntimeError("Connection broken")

        return reply

    def publish_gps_message(self, position):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = position[0]
        msg.pose.pose.position.y = position[1]
        msg.pose.pose.position.z = position[2]
        msg.pose.pose.orientation.x = 1  # identity quaternion
        msg.pose.pose.orientation.y = 0
        msg.pose.pose.orientation.z = 0
        msg.pose.pose.orientation.w = 0
        msg.pose.covariance = [self.cov_x, 0, 0, 0, 0, 0,  # covariance on x
                               0, self.cov_y, 0, 0, 0, 0,  # covariance on y
                               0, 0, self.cov_z, 0, 0, 0,  # covariance on z
                               0, 0, 0, self.cov_rot, 0, 0,  # large covariance on rot x
                               0, 0, 0, 0, self.cov_rot, 0,  # large covariance on rot y
                               0, 0, 0, 0, 0, self.cov_rot]  # large covariance on rot z
        self.gps_pub.publish(msg)

    def loop(self):
        while not rospy.is_shutdown():
            loc = self.get_marker5_position()
            # self.publish_gps_message(position)
            print loc
            self.rate.sleep()

        self.sock.close()

if __name__ == '__main__':
    rospy.init_node('markerlocator_odometry_node')

    try:
        node = MarkerLocatorOdometryNode()
    except rospy.ROSInterruptException:
        pass
