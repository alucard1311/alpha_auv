#!/usr/bin/python3

#from sympy import im
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf
from sbg_driver.msg import SbgEkfQuat
from sbg_driver.msg import SbgImuData
from sensor_msgs.msg import Imu



rpy_publisher =  rospy.Publisher('/auv3/orientation',Vector3,queue_size=10)
rpy_msg = Vector3()
accel = Vector3()
vel = Vector3()
pose_oreintation_publisher = rospy.Publisher('/auv3/imu', Imu,queue_size =10)



def imu_callback(msg):
    global accel
    global vel
    
    accel = msg.accel
    vel = msg.gyro
    

    
    
def ekf_callback(msg):
    imu_msg= Imu()
    imu_msg.header.stamp = rospy.Time.now()
    imu_msg.header.frame_id = "imu_link"
	
	
	
   



    imu_msg.orientation.x = msg.quaternion.x
    imu_msg.orientation.y = msg.quaternion.y
    imu_msg.orientation.z = msg.quaternion.z
    imu_msg.orientation.w = msg.quaternion.w
 
    imu_msg.angular_velocity = vel
    imu_msg.linear_acceleration = accel


    quaternion = (msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w)
    euler_data = tf.transformations.euler_from_quaternion(quaternion)
    rpy_msg.x = euler_data[0]  # roll
    rpy_msg.y = euler_data[1]  # pitch
    rpy_msg.z = euler_data[2]  # yaw

    print("roll: " + str(euler_data[0]))
    print("pitch: " + str(euler_data[1]))
    print("yaw:  "+ str(euler_data[2]))

    rpy_publisher.publish(rpy_msg)
    pose_oreintation_publisher.publish(imu_msg)

def main():
    rospy.init_node('euler_node',anonymous = True)
    rospy.Subscriber('/sbg/ekf_quat',SbgEkfQuat,ekf_callback)
    rospy.Subscriber('/sbg/imu_data',SbgImuData,imu_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSinterruptExeption:
        print("Error!")


