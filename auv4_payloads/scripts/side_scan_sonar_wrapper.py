#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
import os

file = "/home/atd/side_scan/record"
def sonar_start_callback(data):
    if(data.data == True):
        if os.path.exists(file):
            print("Already exists,delete and try again")

        else:
            f = open("/home/atd/side_scan/record", 'w')
            
def sonar_end_callback(data):
    if(data.data == True):
        if os.path.exists(file):
            print("File exists! Deleting")
            os.remove(file)
        else:
            print("No such file exists")
            
            


if __name__ == "__main__":
    rospy.init_node("auv3_side_scan_sonar")
    rospy.Subscriber("auv3/sonar/start", Bool, sonar_start_callback)
    rospy.Subscriber("auv3/sonar/end", Bool, sonar_end_callback)

    rospy.spin()
