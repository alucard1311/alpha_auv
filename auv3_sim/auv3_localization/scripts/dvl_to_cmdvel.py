#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from waterlinked_a50_ros_driver.msg import DVL

rospy.init_node("dvl_twist_node")
DVL_FRAME_ID = "dvl_link"

cmd_vel_publisher = rospy.Publisher("/twist_velocity", TwistWithCovarianceStamped, queue_size=1)

cmd_vel_msg = TwistWithCovarianceStamped()
cmd_vel_msg.header.frame_id = DVL_FRAME_ID
# cmd_vel_msg.twist.twist.angular = None

def dvlCallback(msg):
    cmd_vel_msg.header.stamp = rospy.Time.now()
    cmd_vel_msg.twist.twist.linear = msg.velocity
    cmd_vel_publisher.publish(cmd_vel_msg)
    
dvl_topic = "/auv3/dvl/data"
dvl_subscriber = rospy.Subscriber(dvl_topic, DVL, dvlCallback)

rate = rospy.Rate(0.1)
while not rospy.is_shutdown():
    rate.sleep()
    