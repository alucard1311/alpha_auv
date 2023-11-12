#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
import socket
from std_msgs.msg import String
from waterlinked_a50_ros_driver.msg import DVL
from waterlinked_a50_ros_driver.msg import DVLBeam
from time import sleep


def connect():
    global s
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(("192.168.194.95", 16171))

        # s.settimeout(1)
    except Exception as e:
        print("Not connected")
        sleep(1)
        connect()


oldJson = ""

theDVL = DVL()
beam0 = DVLBeam()
beam1 = DVLBeam()
beam2 = DVLBeam()
beam3 = DVLBeam()


def getData():
    global oldJson, s
    raw_data = ""

    while not '\n' in raw_data:
        try:
            rec = s.recv(1)

            if len(rec) == 0:
                rospy.logerr("Socket closed by the DVL, reopening")
                connect()
                continue
        except socket.timeout as err:
            rospy.logerr(
                "Lost connection with the DVL, reinitiating the connection: {}".format(err))
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


def dvl_data_publisher():
    pub_raw = rospy.Publisher('auv3/dvl/json_data', String, queue_size=10)
    pub = rospy.Publisher('auv3/dvl/data', DVL, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        raw_data = getData()
        data = json.loads(raw_data)
        print("=============="+ str(data['vx']))

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

        rate.sleep()


if __name__ == "__main__":

    # raw_data = ""
    # oldJson = ""
    # s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # try:
    #     s.connect(("192.168.194.95", 16171))

    #     # s.settimeout(1)
    # except Exception as e:
    #     print("Not connected")
    # data = s.recv(8192)
    # data = data.decode()
    # raw_data = raw_data + data
    # # raw_data = oldJson + raw_data
    # # oldJson = ""
    # print(raw_data)
    global s
    rospy.init_node('a50_pub', anonymous=True)

    connect()
    dvl_data_publisher()
