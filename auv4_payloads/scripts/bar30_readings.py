#!/usr/bin/python3

import ms5837
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped


pressure_pub = rospy.Publisher("auv3/pressure", Float32, queue_size=10)
temparature_pub = rospy.Publisher("auv3/temperature", Float32, queue_size=10)
depth_pub = rospy.Publisher("auv3/depth", Float32, queue_size=10)
pose_depth_pub = rospy.Publisher("auv3/pose/depth", PoseWithCovarianceStamped,queue_size =10)

# ms5837 object

depth_data = 0.0

bar30 = ms5837.MS5837(ms5837.MODEL_02BA, 1)   # using BUS 1(3,5 pins)
bar30.init()

def pressure_publisher(pressure):
    pressure_pub.publish(pressure)


def temparature_publisher(temp):
    temparature_pub.publish(temp)

def depth_publisher(depth):
    depth_pub.publish(depth)
    print(f"Depth Data: {depth}")

def pressure_sesnor_node():
    global depth_data
    if bar30.read(ms5837.OSR_512):
        bar30.setFluidDensity(1000)
        pressure = bar30.pressure()
        temp = bar30.temperature()
        depth_data = bar30.depth()
        pressure_publisher(pressure)
        temparature_publisher(temp)
        depth_publisher(depth_data)
        pose_depth_publisher(depth_data)
        
    else:
        raise Exception("Sensor read failed")

def pose_depth_publisher(data):
    pose_depth_data = PoseWithCovarianceStamped()
    pose_depth_data.header.stamp = rospy.Time()
    pose_depth_data.header.frame_id = "pressure_sensor_link"
    pose_depth_data.pose.pose.position.z = data

if __name__ == "__main__":
    rospy.init_node('auv3_bar30')
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        try:
            pressure_sesnor_node()
        except Exception as e:
            print(e)
            pressure_sesnor_node()
