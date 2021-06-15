#!/usr/bin/env python3
import rospy
from custom_msg.msg import obavoid_flag_x_z
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from custom_msg.msg import g2g_x_z
from custom_msg.msg import gui_msg

gui_message = gui_msg()
n = 0
def fnc_to_decide_oba_or_g2g(data):
    gui_message.flag_ob_avoid_or_g2g = data.flag


def distace_callback_fnc(harsh):
    gui_message.distance=harsh.data


def flag_callback(harsh):
    if(harsh.data+1 <= n):
        gui_message.goal_no=harsh.data +1

def location_in_x_y(msg):
    gui_message.location_x=msg.x
    gui_message.location_y=msg.z
    gui_msg_pub.publish(gui_message)

def total_goal(msg):
    global n
    n = len(msg.data)

rospy.init_node('gui_messages', anonymous=True)
obavoid_sub = rospy.Subscriber('/obavoid_flag_x_z_topic',obavoid_flag_x_z ,fnc_to_decide_oba_or_g2g)
distance_sub = rospy.Subscriber('/distance_to_goal_topic', Float64,distace_callback_fnc)
flag_sub = rospy.Subscriber('/flag_topic_for_mutilple_goal', Int32,flag_callback)
location_x_y=rospy.Subscriber('/location_in_x_y',g2g_x_z,location_in_x_y)
lat_sub = rospy.Subscriber('/goal_latitude', Float64MultiArray,total_goal)

gui_msg_pub = rospy.Publisher('/gui_msg_topic', gui_msg,latch=True,queue_size=10)
rospy.spin()
