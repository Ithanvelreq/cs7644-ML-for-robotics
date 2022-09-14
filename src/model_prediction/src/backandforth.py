#!/usr/bin/env python
import rospy
from math import sin,cos,pi
from geometry_msgs.msg import Twist

rospy.init_node('backandforth')
cpub = rospy.Publisher("/sim_ros_interface/drone/cmd_vel",Twist,queue_size=1)

rate = rospy.Rate(10)
t0=rospy.Time.now().to_sec()
while not rospy.is_shutdown():
    t=rospy.Time.now().to_sec()
    twist=Twist()
    if cos((t-t0)*pi/3) >= 0:
        twist.linear.x=0.5
    else:
        twist.linear.x=-0.5
    cpub.publish(twist);
    rate.sleep()

