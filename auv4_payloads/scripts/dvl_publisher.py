#!/usr/bin/env python3
import socket
import json
import rospy
from time import sleep
from std_msgs.msg import String, Float64
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
import select
import math

from geometry_msgs.msg import TwistWithCovarianceStamped


def connect():
	global s, TCP_IP, TCP_PORT
	try:
		s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		s.connect((TCP_IP, TCP_PORT))
		s.settimeout(1)
	except socket.error as err:
		rospy.logerr("No route to host, DVL might be booting? {}".format(err))
		sleep(1)
		connect()

oldJson = ""

theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()

velocity_twist_msg = TwistWithCovarianceStamped()
def getData():
	global oldJson, s
	raw_data = ""

	while not '\n' in raw_data:
		try:
			rec = s.recv(1) # Add timeout for that
			if len(rec) == 0:
				rospy.logerr("Socket closed by the DVL, reopening")
				connect()
				continue
		except socket.timeout as err:
			rospy.logerr("Lost connection with the DVL, reinitiating the connection: {}".format(err))
			connect()
			continue
		rec = rec.decode()	
		raw_data = raw_data + rec
	raw_data = oldJson + raw_data
	oldJson = ""
	raw_data = raw_data.split('\n')
	oldJson = raw_data[1]
	raw_data = raw_data[0]
	return raw_data


def publisher():
	pub_raw = rospy.Publisher('/auv3/dvl/json_data', String, queue_size=10)
	pub = rospy.Publisher('/auv3/dvl/data', DVL, queue_size=10)
	velocity_pub = rospy.Publisher('auv3/velocity',Float64,queue_size=10)
	velocity_twist_pub = rospy.Publisher('auv3/twist/velocity',TwistWithCovarianceStamped,queue_size=10)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		raw_data = getData()
		data = json.loads(raw_data)

		# edit: the logic in the original version can't actually publish the raw data
		# we slightly change the if else statement so now
		# do_log_raw_data is true: publish the raw data to /dvl/json_data topic, fill in theDVL using velocity data and publish to dvl/data topic
		# do_log_raw_data is true: only fill in theDVL using velocity data and publish to dvl/data topic

		if do_log_raw_data:
			rospy.loginfo(raw_data)
			pub_raw.publish(raw_data)
			if data["type"] != "velocity":
				continue
		else:
			if data["type"] != "velocity":
				continue
			pub_raw.publish(raw_data)

		theDVL.header.stamp = rospy.Time.now()
		theDVL.header.frame_id = "dvl_link"
		theDVL.time = data["time"]
		theDVL.velocity.x = data["vx"]
		theDVL.velocity.y = data["vy"]
		theDVL.velocity.z = data["vz"]
		theDVL.fom = data["fom"]
		theDVL.altitude = data["altitude"]
		theDVL.velocity_valid = data["velocity_valid"]
		theDVL.status = data["status"]
		theDVL.form = data["format"]

		vx = data["vx"]
		vy = data["vy"]
		vz = data["vx"]
		vf = math.sqrt(vx**2+vy**2+vz**2)
		
		print(vf)
  
  
		velocity_twist_msg.header.stamp = rospy.Time()
		velocity_twist_msg.header.frame_id = "dvl_link"
		velocity_twist_msg.twist.twist.linear.x = vx
		velocity_twist_msg.twist.twist.linear.y = vy
		velocity_twist_msg.twist.twist.linear.z = vz

		beam0.id = data["transducers"][0]["id"]
		beam0.velocity = data["transducers"][0]["velocity"]
		beam0.distance = data["transducers"][0]["distance"]
		beam0.rssi = data["transducers"][0]["rssi"]
		beam0.nsd = data["transducers"][0]["nsd"]
		beam0.valid = data["transducers"][0]["beam_valid"]

		beam1.id = data["transducers"][1]["id"]
		beam1.velocity = data["transducers"][1]["velocity"]
		beam1.distance = data["transducers"][1]["distance"]
		beam1.rssi = data["transducers"][1]["rssi"]
		beam1.nsd = data["transducers"][1]["nsd"]
		beam1.valid = data["transducers"][1]["beam_valid"]

		beam2.id = data["transducers"][2]["id"]
		beam2.velocity = data["transducers"][2]["velocity"]
		beam2.distance = data["transducers"][2]["distance"]
		beam2.rssi = data["transducers"][2]["rssi"]
		beam2.nsd = data["transducers"][2]["nsd"]
		beam2.valid = data["transducers"][2]["beam_valid"]

		beam3.id = data["transducers"][3]["id"]
		beam3.velocity = data["transducers"][3]["velocity"]
		beam3.distance = data["transducers"][3]["distance"]
		beam3.rssi = data["transducers"][3]["rssi"]
		beam3.nsd = data["transducers"][3]["nsd"]
		beam3.valid = data["transducers"][3]["beam_valid"]

		theDVL.beams = [beam0, beam1, beam2, beam3]

		pub.publish(theDVL)
		velocity_pub.publish(vf)
		velocity_twist_pub.publish(velocity_twist_msg)
		#print("velocity = "+str(data['fom']))

		rate.sleep()

if __name__ == '__main__':
	global s, TCP_IP, TCP_PORT, do_log_raw_data
	rospy.init_node('a50_pub', anonymous=True)
	TCP_IP = rospy.get_param("~ip", "192.168.194.95")
	TCP_PORT = rospy.get_param("~port", 16171)
	do_log_raw_data = rospy.get_param("~do_log_raw_data", False)
	connect()
	try:
		publisher()
	except rospy.ROSInterruptException:
		s.close()