#!/usr/bin/python3

"""
    This file is part of AUV V-4.
    Author: Vinay Naidu
    Email: vinay.naidu1311@gmail.com
    Year: 2023
    Copyright (C) 2023 Autonomous and Undersea Subsystems Division
    
"""
   
    
 


import rospy
from mvp_msgs.msg import ControlProcess
from geometry_msgs.msg import PoseStamped, TwistStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from freefloating_gazebo.srv import ControlType, ControlTypeRequest


pose_set_pub = rospy.Publisher('/auv3/body_position_setpoint',PoseStamped,queue_size=10)
vel_set_pub = rospy.Publisher('/auv3/body_velocity_setpoint',TwistStamped,queue_size=10)




def position_setpoint_publisher(x,y,z,w, depth):
    pose_setpoint_msg = PoseStamped()
    pose_setpoint_msg.header.frame_id = "world"
    pose_setpoint_msg.pose.position.z = depth
    pose_setpoint_msg.pose.orientation.x = x
    pose_setpoint_msg.pose.orientation.y = y
    pose_setpoint_msg.pose.orientation.z = z
    pose_setpoint_msg.pose.orientation.w = w
    pose_set_pub.publish(pose_setpoint_msg)
    
def velocity_setpoint_publisher(vel):
    vel_setpoint_msg = TwistStamped()
    vel_setpoint_msg.header.frame_id = "base_link"
    vel_setpoint_msg.twist.linear.x = vel
    vel_set_pub.publish(vel_setpoint_msg)
    

    

def navigation_controller(msg):
    if msg.control_mode == 'flight':
        print("Recieved Mission, Executing")
        quat = quaternion_from_euler(msg.orientation.x,msg.orientation.y,msg.orientation.z)
        position_setpoint_publisher(quat[0],quat[1],quat[2],quat[3],msg.position.z)
        velocity_setpoint_publisher(msg.velocity.x)
    elif msg.control_mode =='idle':
        print('idle mode, Standing by')
        velocity_setpoint_publisher(0)
        
        
        


if __name__ == "__main__":
    rospy.init_node("navigation_controller")
    rospy.wait_for_service('/auv3/controllers/body_velocity_control')
    rospy.wait_for_service('/auv3/controllers/body_position_control')
    
    
    rospy.Subscriber("/auv3/controller/process/set_point", ControlProcess, navigation_controller)
    velocity_service = rospy.ServiceProxy('/auv3/controllers/body_velocity_control',ControlType)
    position_service = rospy.ServiceProxy('/auv3/controllers/body_position_control',ControlType)
    
    #For velocity
    velocity_request = ControlTypeRequest()    
    velocity_request.axes = ['x']
    vel_res = velocity_service(velocity_request)
    
    #For Position
    position_request = ControlTypeRequest()
    position_request.axes = ["yaw", "z"]
    pos_res = position_service(position_request)
    
    
    rospy.spin()