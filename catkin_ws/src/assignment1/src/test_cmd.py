#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan




cmd = Twist()
rospy.init_node('topics_quiz')
cmd.linear.y = 0.5
# Publisher definition
# We publish in /cmd_vel a Twist message
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(2)

while not rospy.is_shutdown():
    cmd.linear.z = 0
    cmd.angular.y = 0
    cmd.angular.x = 0
    cmd.linear.y = 0
    cmd.linear.x=0
    cmd.angular.z=0
    pub.publish(cmd)
    rate.sleep()