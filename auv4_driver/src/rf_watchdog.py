#!/usr/bin/python3

import rospy
import socket
from std_msgs.msg import Bool,String

rospy.init_node("auv3_watchdogs")
alert_pub = rospy.Publisher("auv3/rf_alert",Bool,queue_size=10)

yaw_pub = rospy.Publisher("/auv3/yaw_msg",String,queue_size=10)
surge_pub = rospy.Publisher("auv3/surge_msg",String,queue_size=10)
pitch_front_pub = rospy.Publisher("/auv3/pitch_front_msg",String,queue_size=10)
pitch_back_pub = rospy.Publisher("/auv3/pitch_back_msg",String,queue_size=10)

host = "192.168.1.50"
port = 9090
timeout_secs = 1

alert_bool = Bool()

while not rospy.is_shutdown():
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(timeout_secs)
    result = sock.connect_ex((host, int(port)))
    
    if(result==0):
        print("Link exists!")
    else:

        print("No Link! Nuetralising thrusters")
        yaw_pub.publish(str(75))
        surge_pub.publish(str(75))
        pitch_front_pub.publish(str(75))
        pitch_back_pub.publish(str(75))
        
        