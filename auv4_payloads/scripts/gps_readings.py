#!/usr/bin/env python3

import time
import rospy
from serial import Serial
from sensor_msgs.msg import NavSatFix
from ublox_gps import UbloxGps
from std_msgs.msg import Bool


# global publishers initialisation
gps_pub = rospy.Publisher("auv3/gps_coordinates",NavSatFix,queue_size =10)
gps_lock_pub = rospy.Publisher("auv3/gps_lock", Bool,queue_size=10)


#create Private Objects fo GPS and Serial 
port = Serial('/dev/ttyTHS1', baudrate=38400, timeout=1)
gps = UbloxGps(port)
gps_lock_bool = Bool()

#private functions declaration
def gps_coordinates_publisher(latitude,longitude):
   # gps_lock_bool.data = True
   # gps_lock_pub.publish(gps_lock_bool)
    gps_msg = NavSatFix()
    gps_msg.header.stamp = rospy.Time()
    gps_msg.header.frame_id = "gps_link"
    gps_msg.latitude = latitude
    gps_msg.longitude = longitude
    print("Latitude: " + str(latitude)+" "+ "Longitude: "+ str(longitude)+ "\n")
    gps_pub.publish(gps_msg)

if __name__ == "__main__":
    rospy.init_node("gps_neo",anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        if port.is_open:
            #print("Serial Port is established!")
            coords = gps.geo_coords()
            #if coords.lat != null:
            lon = coords.lon
            lat = coords.lat
            gps_coordinates_publisher(lat,lon)
            #else:
             #   gps_lock_bool.data = False
              #  gps_lock_pub.publish()
               # print("No Lock")
        else:
            print("Serail Communication is not established!!")
            exit(1)
