#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from custom_msg.msg import ultrasonic_msg

message= ultrasonic_msg()

def s1(msg):
    message.r1=msg.range
def s2(msg):
    message.r2=msg.range
def s3(msg,pub1):
    message.r3=msg.range
    pub1.publish(message)

rospy.init_node('ultrasonic_data', anonymous=True)
pub1 = rospy.Publisher('/ultrasonic_msg_topic', ultrasonic_msg, queue_size=10)	
sub1 = rospy.Subscriber('/bot/ir_front_left1',Range,s1)
sub2 = rospy.Subscriber('/bot/ir_front_middle',Range,s2)
sub3 = rospy.Subscriber('/bot/ir_front_right2',Range,s3,pub1)
# /bot/ir_front_right1
# bot/ir_front_left2
rospy.spin()
