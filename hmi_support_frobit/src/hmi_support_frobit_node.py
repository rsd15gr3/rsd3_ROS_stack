#!/usr/bin/env python

"""
This node enables the HMI to control Mobile Robot (Frobit) and its Tipper

subscribing:
- ui/str_control_frobit

publishing:
- /fmSafe/deadman       (20Hz, Frobit's deadman signal, always True, BoolStamped)
- /fmPlan/automode      (20Hz, Frobit's automode state, IntStamped)
- /fmCommand/cmd_vel    (20Hz if automode=0, Frobit's manual control, TwistStamped)
- /ui/tipper_automode   (20Hz, Tipper's automode state, BoolStamped)
- /ui/tipper_position   (on change, Tipper's manul control, FloatStamped : 0.0 .. bottom, 1.0 .. top)
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from msgs.msg import BoolStamped, IntStamped, FloatStamped


class Frobit():

    def __init__(self, node):
        """ Frobit Instance Initialization """

        ''' Register self '''
        self.node = node

        ''' Read parameters from launchfile '''
        self.tp_automode = rospy.get_param('~mr_tp_automode', '/fmPlan/automode')
        self.tp_deadman = rospy.get_param('~mr_tp_deadman', '/fmSafe/deadman')
        self.tp_cmd_vel = rospy.get_param('~mr_tp_cmd_vel', '/fmCommand/cmd_vel')
        self.vel_lin_max = rospy.get_param('~mr_max_linear_velocity', 1)                    # [m/s]     TODO: tune max velocities
        self.vel_ang_max = rospy.get_param('~mr_max_angular_velocity', 1)                   # [rad/s]

        ''' Setup topics '''
        # Setup Mobile Robot automode publish topic
        self.tp_automode_message = IntStamped()
        self.tp_automode_message.data = 0
        self.tp_automode_publisher = rospy.Publisher(self.tp_automode, IntStamped, queue_size=1)

        # Setup Mobile Robot deadman publish topic
        self.tp_deadman_message = BoolStamped()
        self.tp_deadman_message.data = True
        self.tp_deadman_publisher = rospy.Publisher(self.tp_deadman, BoolStamped, queue_size=1)

        # Setup Mobile Robot manual velocity topic
        self.tp_cmd_vel_message = TwistStamped()
        self.tp_cmd_vel_publisher = rospy.Publisher(self.tp_cmd_vel, TwistStamped, queue_size=1)

        ''' Variables '''
        self.vel_lin = 0.0
        self.vel_ang = 0.0
        self.js_x = None
        self.js_y = None
        self.js_x0 = None
        self.js_y0 = None
        self.x_diff_max = 100
        self.y_diff_max = 100

    def decode_control(self, data):
        if data[1] == 'joystick':
            positions = data[2].split(';')
            self.js_x0 = float(positions[0])
            self.js_y0 = float(positions[1])
            self.js_x = float(positions[2])
            self.js_y = float(positions[3])

            x_diff = abs(self.js_x-self.js_x0)
            y_diff = abs(self.js_y-self.js_y0)

            self.vel_ang = (x_diff/self.x_diff_max)*self.vel_ang_max
            self.vel_lin = (y_diff/self.y_diff_max)*self.vel_lin_max

            if self.js_x > self.js_x0:
                self.vel_ang *= -1

            if self.js_y > self.js_y0:
                self.vel_lin *= -1

            # Velocity limits
            self.vel_lin = min(self.vel_lin_max, max(self.vel_lin, -self.vel_lin_max))
            self.vel_ang = min(self.vel_ang_max, max(self.vel_ang, -self.vel_ang_max))
        elif data[1] == 'stop':
            self.stop_frobit()
        elif data[1] == 'mode':
            if data[2] == 'auto':
                self.tp_automode_message.data = 1
            elif data[2] == 'manual':
                self.stop_frobit()
                self.tp_automode_message.data = 0

    def publish_tp_automode_message(self):
        self.tp_automode_message.header.stamp = rospy.get_rostime()
        self.tp_automode_publisher.publish (self.tp_automode_message)

    def publish_tp_deadman_message(self):
        self.tp_deadman_message.header.stamp = rospy.get_rostime()
        self.tp_deadman_publisher.publish(self.tp_deadman_message)

    def publish_tp_cmd_vel_message(self):
        self.tp_cmd_vel_message.header.stamp = rospy.Time.now()
        self.tp_cmd_vel_message.twist.linear.x = self.vel_lin
        self.tp_cmd_vel_message.twist.angular.z = self.vel_ang
        self.tp_cmd_vel_publisher.publish(self.tp_cmd_vel_message)

    def stop_frobit(self):
        self.vel_ang = 0.0
        self.vel_lin = 0.0


class Tipper():

    def __init__(self, node):
        """ Tipper Instance Initialization """

        ''' Register self '''
        self.node = node

        ''' Read parameters from launchfile '''
        self.tp_automode = rospy.get_param('~tipper_tp_automode', '/ui/tipper_automode')
        self.tp_position = rospy.get_param('~tipper_tp_position', '/ui/tipper_position')
        self.position_top = rospy.get_param('~tipper_position_top', 1.0)
        self.position_bottom = rospy.get_param('~tipper_position_bottom', 0)
        self.position_step = rospy.get_param('~tipper_position_step', 2)

        ''' Setup topics '''
        # Setup Tipper automode publish topic
        self.tp_automode_message = BoolStamped()
        self.tp_automode_message.data = False
        self.tp_automode_publisher = rospy.Publisher(self.tp_automode, BoolStamped, queue_size=1)

        # Setup Tipper position publish topic
        self.tp_position_message = IntStamped()
        self.tp_position_message.data = self.position_bottom
        self.tp_position_publisher = rospy.Publisher(self.tp_position, IntStamped, queue_size=1)

    def decode_control(self, data):
        if data[1] == 'move':
            if data[2] == 'up':
                self.tp_position_message.data = min(self.tp_position_message.data+self.position_step, self.position_top)
            elif data[2] == 'down':
                self.tp_position_message.data = max(self.tp_position_message.data-self.position_step, self.position_bottom)
            self.publish_tp_position_message()
        elif data[1] == 'mode':
            if data[2] == 'auto':
                self.tp_automode_message.data = True
            elif data[2] == 'manual':
                self.tp_position_message.data = 0.0
                self.tp_automode_message.data = False

    def publish_tp_automode_message(self):
        self.tp_automode_message.header.stamp = rospy.get_rostime()
        self.tp_automode_publisher.publish (self.tp_automode_message)

    def publish_tp_position_message(self):
        self.tp_position_message.header.stamp = rospy.get_rostime()
        self.tp_position_publisher.publish (self.tp_position_message)


class Node():

    def __init__(self):
        """ Node Instance Initialization """

        ''' Init Frobit's functionalities '''
        self.frobit = Frobit(node=self)

        ''' Init Tipper's functionalities '''
        self.tipper = Tipper(node=self)

        ''' Init /ui/str_control_frobit topic '''
        self.tp_ui_str_control = rospy.get_param('~tp_ui_str_control', '/ui/str_control_frobit')
        rospy.Subscriber(self.tp_ui_str_control, String, self.on_topic_ui_str_control)

        ''' Publishing rate '''
        self.publishing_rate = rospy.Rate(rospy.get_param('~publishing_rate', 20))          # [Hz]

        # Loop function
        self.keep_publishing()

    def on_topic_ui_str_control(self, msg):
        data = msg.data.split('_')
        if data[0] == 'mr':
            self.frobit.decode_control(data=data)
        elif data[0] == 'tipper':
            self.tipper.decode_control(data=data)

    def keep_publishing(self):
        while not rospy.is_shutdown():
            #self.frobit.publish_tp_deadman_message()
            self.frobit.publish_tp_automode_message()
            self.tipper.publish_tp_automode_message()
            if self.frobit.tp_automode_message.data == 0:
                self.frobit.publish_tp_cmd_vel_message()
            self.publishing_rate.sleep()

if __name__ == '__main__':
    """ The program starts here by naming the node and initializing it. """

    rospy.init_node('hmi_support_frobit_node')
    try:
        Node()
    except rospy.ROSInterruptException:
        pass
