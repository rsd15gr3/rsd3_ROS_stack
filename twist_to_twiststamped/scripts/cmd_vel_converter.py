#!/usr/bin/env python
#/****************************************************************************
# FroboMind cmd_vel_converter.py
# Copyright (c) 2011-2015, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
2013-11-06 Kjeld Jensen profiled code and added launch parameters for the topics
	(currently defaults to the previous static names though this is conflicting with
	the fmNaming convention).
2015-03-05 KJ Added queue_size to rospy.Publisher calls (Indigo compatiblity)
2015-08-20 KJ Removed check for deadman signal, this is now handled in fmSafety.
"""

import rospy
from geometry_msgs.msg import TwistStamped,Twist
from std_msgs.msg import Bool

class CmdVelConverter():
	"""
		Converter for using FroboMind with stage. 
		Takes TwistStamped message from /fmSignals/cmd_vel and parses as Twist message on /cmd_vel
	"""
	def __init__(self):
		# Init node
		topic_tw_stamped = rospy.get_param("~cmd_vel_pub", "/fmCommand/cmd_vel")
		topic_tw = rospy.get_param("~cmd_vel_sub", "/cmd_vel")
		self.twist_pub = rospy.Publisher(topic_tw_stamped, TwistStamped, queue_size=1)
		self.twist_sub = rospy.Subscriber(topic_tw, Twist, self.onTwist )
		
	def onTwist(self,msg):
		out_msg = TwistStamped()
		out_msg.header.stamp = rospy.Time.now()
		out_msg.twist = msg
		self.twist_pub.publish(out_msg)

if __name__ == '__main__':
	rospy.init_node('cmd_vel_converter')
	node = CmdVelConverter()
	rospy.spin()
	
