#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from custom_msg.msg import obavoid_flag_x_z
from custom_msg.msg import g2g_x_z

vel_msg1 = Twist()
vel_msg2 = Twist()
flag=[0]
def f1(msg1):
    vel_msg1.linear.x = msg1.x
    vel_msg1.angular.z = msg1.z

def f2(msg2,velocity_publisher):
    global flag
    vel_msg2.linear.x = msg2.x
    vel_msg2.angular.z = msg2.z
    flag=msg2.flag
    f3(velocity_publisher)

def f3(velocity_publisher): 
    if flag==1:
	    velocity_publisher.publish(vel_msg1)
	    print(" flag=1 going forward g2g ")
    else:
	    velocity_publisher.publish(vel_msg2)
	    print("flag =0  ob avoid")

rospy.init_node('g2g_with_obavoid', anonymous=True)
velocity_publisher = rospy.Publisher('/cmd_vel_auto', Twist, queue_size=50)	
obavoid_sub = rospy.Subscriber('/g2g_x_z_topic', g2g_x_z,f1)
obavoid_sub = rospy.Subscriber('/obavoid_flag_x_z_topic', obavoid_flag_x_z,f2,velocity_publisher)
rospy.spin()
