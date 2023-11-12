#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped, Twist

def cmd_vel_callback(msg):
    twist_stamped = TwistStamped()
    twist_stamped.header.stamp = rospy.Time.now()
    twist_stamped.header.frame_id = 'base_link' # or whatever your frame id is
    twist_stamped.twist = msg
    twist_stamped_pub.publish(twist_stamped)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_cmd_vel_stamped')
    twist_stamped_pub = rospy.Publisher('/auv3/body_velocity_setpoint', TwistStamped, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rospy.spin()
