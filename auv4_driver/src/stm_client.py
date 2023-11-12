#!/usr/bin/python3

import socket
import rospy
from std_msgs.msg import String, Bool
import time




yaw1AddressPort   = ("10.42.0.55", 10)
surge1AddressPort   = ("10.42.0.55", 15)
pitch_front1AddressPort   = ("10.42.0.55", 7)
pitch_back1AddressPort   = ("10.42.0.55", 20)



UDPClientSocket1 = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)




def yaw_sub_callback(msg):
	print("Yaw Msg = "+ msg.data)
	UDPClientSocket1.sendto(str.encode(msg.data), yaw1AddressPort)
	
	
	
    
def surge_sub_callback(msg):
    print("Surge Msg = "+ msg.data)
    UDPClientSocket1.sendto(str.encode(msg.data), surge1AddressPort)

def pitch_front_callback(msg):
    print('pitch_front_msg='+msg.data)
    UDPClientSocket1.sendto(str.encode(msg.data), pitch_front1AddressPort)

def pitch_back_callback(msg):
    print('pitch_back_msg='+msg.data)
    UDPClientSocket1.sendto(str.encode(msg.data), pitch_back1AddressPort)


if __name__ == "__main__":
	
    rospy.init_node("stm_client",anonymous=True)
    print("STM_Client Initialised!!!")
    yaw_sub = rospy.Subscriber('/auv3/yaw_msg',String,yaw_sub_callback)
    surge_sub = rospy.Subscriber('/auv3/surge_msg',String,surge_sub_callback)
    pitch_front_pub = rospy.Subscriber('/auv3/pitch_front_msg',String,pitch_front_callback)
    pitch_back_pub = rospy.Subscriber('/auv3/pitch_back_msg',String,pitch_back_callback)
    #stm_link_lock = rospy.Publisher('auv3/stm_lock', Bool, queue_size=10)



    rospy.spin()