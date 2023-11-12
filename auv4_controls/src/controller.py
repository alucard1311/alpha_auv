#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32, String, Float64
from geometry_msgs.msg import Vector3
from math import copysign
import time
import math
# publishers
pitch_back_pub = rospy.Publisher('auv3/pitch_back_msg', String, queue_size=10)
pitch_front_pub = rospy.Publisher('auv3/pitch_front_msg', String, queue_size=10)
surge_pub = rospy.Publisher('/auv3/surge_msg',String,queue_size=10)

plant_data_pub = rospy.Publisher('auv3/plant/depth', Float64, queue_size=10)
pitch_plant_pub = rospy.Publisher('auv3/plant/pitch_data', Float64, queue_size=10)
yaw_thrust_pub = rospy.Publisher('auv3/yaw_msg', String, queue_size=10)
yaw_plant_pub = rospy.Publisher("auv3/plant/yaw", Float64, queue_size=10)

pi = math.pi


# plants
pitch_plant_msg = 0.0
depth_plant_msg = 0.0
yaw_plant_msg = 0.0
velocity_plant_msg = 0.0

# setpoints
depth_setpoint_msg = 0.0
yaw_setpoint_msg = 0.0
velocity_setpoint = 0.0


def depth_callback(data):
    depth_plant_msg = data.data
    #print("sending pressure data to controller!")
    plant_data_pub.publish(depth_plant_msg)


def depth_setpoint_callback(data):
    global depth_setpoint_msg
    depth_setpoint_msg = data.data
    print("setpoint is =" + str(depth_setpoint_msg))


def yaw_setpoint_callback(data):
    global yaw_setpoint_msg
    yaw_setpoint_msg = data.data
    print("Yaw_setpoint = " + str(yaw_setpoint_msg))


def compare_angles(x_plant, x_setpoint):
    d = abs(x_setpoint-x_plant)
    if(d > pi):
        d -= (2*pi)
    return copysign(d, (x_plant-x_setpoint))


def pitch_effort_callback(data):
    global pitch_effort_msg
    pitch_effort_msg = data.data
    #print("Pitch Effort Data = " + str(pitch_effort_msg))


def orientation_callback(data):
    global pitch_plant_msg
    global yaw_plant_msg
    pitch_plant_msg = data.y
    yaw_plant_msg = data.z
    yaw_plant_pub.publish(yaw_plant_msg)

    pitch_plant_pub.publish(pitch_plant_msg)


'''Main Control logics are here'''

# Depth and Pitch Controll


def depth_controller(data):
    depth_effort_msg = data.data
    thruster_back_msg = 75
    thruster_front_msg = 75
    #yaw_msg = 75

    # if depth_plant_msg > depth_setpoint_msg:
    #     print("auv3 is coming up")
    # else:
    #     print("auv3 is going down")

    # if pitch_plant_msg < 0:
    #     print("auv nose is down")
    # else:
    #     print("auv nose is up")
    #print("Depth Effort== "+ str(depth_effort_msg) )

    if(depth_plant_msg < depth_setpoint_msg):
        thruster_back_msg -= depth_effort_msg
        thruster_front_msg -= depth_effort_msg
    if(pitch_plant_msg > 0):
        # print("I am here"
        thruster_front_msg -= 3
        # print(thruster_back_msg)
    if(pitch_plant_msg < 0):
        thruster_back_msg -= 3

    # diff = compare_angles(yaw_plant_msg, yaw_setpoint_msg)

    # if(diff < 0):
    #     yaw_msg += 4
    # if(diff > 0):
    #     yaw_msg -= 4

    pitch_back_pub.publish(str(round(thruster_back_msg)))
    pitch_front_pub.publish(str(round(thruster_front_msg)))
    # yaw_thrust_pub.publish(str(yaw_msg))
    time.sleep(0.1)

# Yaw Control


def yaw_controller(data):
    yaw_msg = 75
    yaw_effort_msg = data.data
    diff = compare_angles(yaw_plant_msg, yaw_setpoint_msg)
    if(diff < 0):
        yaw_msg += yaw_effort_msg
    if(diff > 0):
        yaw_msg -= yaw_effort_msg
    yaw_thrust_pub.publish(str(yaw_msg))
    time.sleep(0.1)


# Velocity Control
def velocity_controller(data):
    surge_msg = 75
    velocity_effort_msg = data.data
    surge_msg+=velocity_effort_msg
    surge_pub.publish(round(surge_msg))

if __name__ == "__main__":
    rospy.init_node('depth_pid_controller', anonymous=True)
    r = rospy.Rate(1)
    # Subscribers

    oreintation_sub = rospy.Subscriber('/auv3/orientation', Vector3, orientation_callback)
    pressure_sub = rospy.Subscriber("/auv3/depth", Float32, depth_callback)
    # velocity_sub = rospy.Subscriber("/auv3/velocity",Float32,velocity_callback)
    
        
    depth_effort_sub = rospy.Subscriber("auv3/controller/depth_effort", Float64, depth_controller)
    depth_setpoint_sub = rospy.Subscriber("auv3/setpoint_depth", Float64, depth_setpoint_callback)
    yaw_setpoint_sub = rospy.Subscriber("auv3/yaw_setpoint", Float64, yaw_setpoint_callback)
    yaw_effort_sub = rospy.Subscriber("auv3/controller/yaw_effort", Float64, yaw_controller)
    velocity_effort_sub = rospy.Subscriber('auv3/velocity_effort',Float64,velocity_controller)
  
    print("Controller Script Initialized!")
    rospy.spin()
