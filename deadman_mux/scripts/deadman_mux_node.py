#!/usr/bin/env python

import rospy
from msgs.msg import BoolStamped

class DeadmanMux():
    hmi_signal = BoolStamped()
    arduino_signal = BoolStamped()

    def __init__(self):
        topic_hmi_deadman = rospy.get_param("hmi_deadman_sub", "/hmi_deadman")
        topic_arduino_deadman = rospy.get_param("arduino_deadman_sub", "/arduino_switch")
        topic_frobit_deadman = rospy.get_param("frobit_deadman_pub", "/fmSafe/deadman")
        self.hmi_sub = rospy.Subscriber(topic_hmi_deadman, BoolStamped, self.hmiCb)
        self.arduinio_sub = rospy.Subscriber(topic_arduino_deadman, BoolStamped, self.arduinoCb)
        self.frobit_pub = rospy.Publisher(topic_frobit_deadman, BoolStamped, queue_size=1)

    def hmiCb(self, msg):
        self.hmi_signal = msg

    def arduinoCb(self, msg):
        self.arduino_signal = msg

    def loop(self):
        out_msg = BoolStamped()
        if(self.hmi_signal.data != True or self.arduino_signal.data != True):
            out_msg.data = False
        else:
            out_msg.data = True

        out_msg.header.stamp = rospy.Time.now()
        self.frobit_pub.publish(out_msg)

if __name__ == '__main__':
    rospy.init_node('deadman_mux')
    node = DeadmanMux()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        node.loop()
        rate.sleep()
