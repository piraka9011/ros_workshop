#!/usr/bin/env python

import rospy
import actionlib
from move_base.msgs import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler, euler_from_quaternion

if __name__ == '__main__':
    rospy.init_node('simple_action_cmd')
    # Create the action client and wait for a connection
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for client to connect...")
    client.wait_for_server()
    rospy.loginfo("Connected!")

    # Create the goal
    # YOU FILL THESE
    target_goal = MoveBaseGoal()
    target_goal.header.frame_id = 'map'
    target_goal.header.stamp = rospy.Time.now()
    # Position has XYZ
    # target_goal.pose.position.x = 0.0
    # target_goal.pose.position.y = 0.0
    # Orientation is a quaternion with XYZW
    # target_goal.pose.orientation.w = 1.0
    # You can also use the quaternion_from_euler function to convert RPY to Quaternion
    # You can also do the inverse with euler_from_quaternion
    # quaternion = quaternion_from_euler(roll, pitch, yaw)
    # x = quaternion[0]
    # y = quaternion[1]
    # z = quaternion[2]
    # w = quaternion[3]

    rospy.loginfo("Sending goal...")
    client.send_goal(target_goal)
    client.wait_for_result()
    result = client.get_result()
    rospy.loginfo("Got the following result: {}".format(result))
