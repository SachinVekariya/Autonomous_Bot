#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from custom_msg.msg import ultrasonic_msg
from custom_msg.msg import obavoid_flag_x_z
message = obavoid_flag_x_z()
[r1,r2,r3]=[0.0,0.0,0.0]
def f1(msg,pub):
	global r1,r2,r3
	r1=msg.r1
	r2=msg.r2
	r3=msg.r3
	ob_avoid(pub)

def forward():
	message.x = 0.7
	message.z = 0
def stop():
	message.x = 0
	message.z = 0
def backward():
	message.x = -0.7
	message.z = 0
def backward_right():
	message.x = -0.2
	message.z = -0.2
def sharp_right_turn(): 
	message.x = 0
	message.z = -0.5
	
def sharp_left_turn():
	message.x = 0
	message.z = 0.5

def soft_right_turn(): 
	message.x = 0.7
	message.z = -0.5
	
def soft_left_turn():
	message.x = 0.7
	message.z = 0.5

def ob_avoid(pub):
	global r1,r2,r3,message
	if r1>2.5 and r2>2.5 and r3>2.5:
		forward()
		message.flag=1
		#print("...going forward")			
	elif r1<2.5 and r2<2.5 and r3<2.5:
		backward()
		message.flag=0
		#print("...going backward")
	elif r1>2.5 and r2>2.5 and r3<2.5:
		soft_left_turn()
		message.flag=0
		#print("...soft left turn")
	elif r1<2.5 and r2>2.5 and r3>2.5:
		soft_right_turn()
		message.flag=0
		#print("...soft right turn")
	elif r1>2.5 and r2<2.5 and r3<2.5:
		sharp_left_turn()
		message.flag=0
		#print("...sharp left turn")
	elif r1<2.5 and r2<2.5 and r3>2.5:
		sharp_right_turn()
		message.flag=0
		#print("...sharp right turn")
	elif r1>2.5 and r2<2.5 and r3>2.5:
		sharp_right_turn()
		message.flag=0
		#print("...sharp right turn")
	elif r1<4 and r2>2.5 and r3<4:
		forward()
		message.flag=1
		#print("***going forward")
	elif r1<1.5 and r2>2.5 and r3<1.5:
		backward_right()
		message.flag=0
		#print("***going backward_right")
   		#print("***going backward_right")
	else:
		forward()
		message.flag=1
		#print("...going forward")		
	pub.publish(message)

rospy.init_node('ob_avoid5', anonymous=True)
pub = rospy.Publisher('/obavoid_flag_x_z_topic', obavoid_flag_x_z, queue_size=50)	
ultrasonic_msg_sub = rospy.Subscriber('/ultrasonic_msg_topic',ultrasonic_msg,f1,pub)
rospy.spin()


