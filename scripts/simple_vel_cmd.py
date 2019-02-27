#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('simple_vel_cmd')
    # Create a publisher
    vel_publisher = rospy.Publisher('/cmd_vel_mux/teleop', Twist, queue_size=10)
    # Create the twist message
    twist = Twist()
    twist.linear.x = 0.5
    # Set a rate to loop at (5 Hz)
    rate = rospy.Rate(5)

    # Loop forever
    while True:
        try:
            vel_publisher.publish(twist)
            rate.sleep()
        except KeyboardInterrupt:
            print("Exiting")
            sys.exit(0)

         