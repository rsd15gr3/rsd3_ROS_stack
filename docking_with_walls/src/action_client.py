#! /usr/bin/env python

import roslib
import rospy
import actionlib

from docking_with_walls.msg import docking_with_wallsAction, docking_with_wallsGoal

if __name__ == '__main__':
    rospy.init_node('action_docking_with_walls_client')
    client = actionlib.SimpleActionClient('docking_with_walls_server', docking_with_wallsAction)
    print client
    client.wait_for_server()

    goal = docking_with_wallsGoal()
    print goal
    # Fill in the goal here
    client.send_goal(goal)
    print client.wait_for_result(rospy.Duration.from_sec(5.0))
