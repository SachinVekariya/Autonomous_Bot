#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose,Quaternion
from sensor_msgs.msg import NavSatFix
from custom_msg.msg import g2g_x_z

msg = g2g_x_z()
lacation_in_x_y=g2g_x_z()  #assume z as y here
class gps_point:
    lat=0.0
    lon=0.0
    theta=0.0
    name="map"
p=gps_point()
roll = pitch = yaw = 0.0
flag = 0
[x,y,angle_diff,distance]=[0,0,0,0]

def current_location(msg):
    
    p.lat=msg.latitude
    p.lon=msg.longitude
    print("current latitude :",p.lat,"current longitude:",p.lon)

def get_imu_angle (msg):
    global roll, pitch, yaw
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    #print("yaw in radian:",yaw)
    yaw=yaw*180/math.pi
    if yaw<0:
	    yaw=yaw+360
    print("yaw in degree:",yaw)
  

def fnc1(gps_pose,args):
    global x,y,gps_angle,yaw,distance,angle_diff
    lacation_in_x_y.x=-gps_pose.position.x
    lacation_in_x_y.z=-gps_pose.position.y
    loaction_in_x_y_pub.publish(lacation_in_x_y)
    x=gps_pose.position.x
    x=-x
    #print("x:",x)
    y=gps_pose.position.y
    y=-y
    #print("y:",y)
    gps_angle= (math.atan(abs(y)/abs(x)))*180/math.pi

    if x>0 and y>0:
	    gps_angle =gps_angle-90
    elif x>0 and y<0:
        gps_angle = -(90+gps_angle)
    elif x<0 and y>0:
	    gps_angle=90-gps_angle
    else:
	    gps_angle =90+gps_angle
    
    print("gps_angle:",gps_angle)

    angle_diff= gps_angle-yaw
    if angle_diff >180:
	    angle_diff -= 360
    elif angle_diff <-180:
	    angle_diff +=360
    #print("angle_diff:",angle_diff)
    
    distance=math.sqrt(math.pow(x,2)+math.pow(y,2))
    #print("distance:",distance)
   
    atnms(args[0],args[1],args[2])

def forward():
	msg.x = 0.7
	msg.z = 0

def right_turn(): 
	msg.x = 0
	msg.z = -0.2
	
def left_turn():
	msg.x = 0
	msg.z = 0.2

def stop():
	msg.x = 0
	msg.z = 0

def atnms(g2g_x_z_pub,flag_pub,distance_pub):
	global angle_diff,flag,distance
	if distance < 1 :
		stop()
		flag=flag+1
		print("stop")
	elif angle_diff>0 and abs(angle_diff)>5:
		left_turn()
		print("left turn")			
	elif angle_diff<0 and abs(angle_diff)>5:
		right_turn()
		print("right turn")
	elif abs(angle_diff)<5:
		forward()
		print("going forward")

	g2g_x_z_pub.publish(msg)
	flag_pub.publish(flag)
	distance_pub.publish(distance)
	
rospy.init_node('g2g_x_z', anonymous=True)
distance_pub = rospy.Publisher('/distance_to_goal_topic', Float64,queue_size=50)
g2g_x_z_pub = rospy.Publisher('/g2g_x_z_topic', g2g_x_z, queue_size=9)
flag_pub = rospy.Publisher('/flag_topic_for_mutilple_goal', Int32,latch=True,queue_size=10)
loaction_in_x_y_pub = rospy.Publisher('/location_in_x_y', g2g_x_z, queue_size=10)

sub=rospy.Subscriber('/gps_fix',NavSatFix,current_location)	
imu_data = rospy.Subscriber('/imu/data',Imu,get_imu_angle)
gps_data = rospy.Subscriber('/distance',Pose,fnc1,(g2g_x_z_pub,flag_pub,distance_pub))

rospy.spin()
