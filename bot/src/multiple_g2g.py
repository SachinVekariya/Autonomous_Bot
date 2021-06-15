#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from custom_msg.msg import lat_long_msg
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
lat_long_msg_1=lat_long_msg()
flag=0
n=0
lat=[]
lon=[]
p=NavSatFix()
def Cloning(li1):
    li_copy = []
    li_copy.extend(li1)
    return li_copy

def flag_callback(msg,args):
    global flag,lat,lon,n
    flag=msg.data
    print("flag=",flag)
    for i in range(n):
        if flag==i:
            lat_long_msg_1.lat=lat[i]
            lat_long_msg_1.long=lon[i]
            p.latitude=lat[i]
            p.longitude=lon[i]
            print("-->current goal no:",flag+1)
        else:
            pass
    args[0].publish(p)    
    args[1].publish(lat_long_msg_1) 

def fnc1(msg):
    global lat,n
    n = len(msg.data)
    lat = Cloning(msg.data)
    
def fnc2(msg):
    global lon
    lon = Cloning(msg.data)


rospy.init_node('multiple_goal_g2g_node', anonymous=True)
entered_arr_lat_sub = rospy.Subscriber('/goal_latitude', Float64MultiArray, fnc1)
entered_arr_long_sub = rospy.Subscriber('/goal_longitude', Float64MultiArray,fnc2)	

lat_long_pub = rospy.Publisher('/Multiple_goal_lat_long_topic', lat_long_msg, latch=True,queue_size=10)
current_goal_pub = rospy.Publisher('/current_goal', NavSatFix, latch=True,queue_size=50)
flag_sub = rospy.Subscriber('/flag_topic_for_mutilple_goal', Int32,flag_callback,(current_goal_pub,lat_long_pub))
rospy.spin()
