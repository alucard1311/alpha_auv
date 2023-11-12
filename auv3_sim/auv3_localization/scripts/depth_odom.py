#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32

rospy.init_node("depth_odom_node")
PRESSURE_FRAME_ID = "pressure_link"

pressure_odom_publisher = rospy.Publisher("/auv3/depth_odom", PoseWithCovarianceStamped, queue_size=1)
pose_msg = PoseWithCovarianceStamped()
pose_msg.header.frame_id = PRESSURE_FRAME_ID

def callback(msg):
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.pose.position.z = msg.data
    pressure_odom_publisher.publish(pose_msg)
    
pressure_topic = "/auv3/depth"
pressure_subscriber = rospy.Subscriber(pressure_topic, Float32, callback)

rate = rospy.Rate(0.1)
while not rospy.is_shutdown():
    rate.sleep()
    