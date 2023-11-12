# AUV3 Description
This repo contains the xacro and mesh files for the AUV3 model. This repo basically describes the AUV3 model.

## Setup and Run
### Create a workspace (optional)
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Clone this Repo
```bash
cd ~/catkin_ws/src
git clone https://github.com/AUV3/auv3_description.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Launch Model in Rviz
```bash
cd ~/catkin_ws
roslaunch auv3_description auv3_rviz.launch
```
---

Communication Establishment steps:
1. Establish a communication interface from our computer to Jetson nano through ssh (Secure Shell). 
2. Then establish a communication from jetson nano to STM and this STM will control the thrusters.

Script Packages Description:
1. auv3_stm_link - This package is responsible for creating separate websocket ports for thrusters (pitch, surge and yaw ).
2. auv3_gcs_link - This package is resposible for linking GCS (Ground Communication System) with jetson nano and linking joystick.
3. auv3_payloads - This package is responsible for launching all the three sensors at a time.
                 
Auv3_payloads scripts Definition:
1. pressure_sensor.py - Only the pressure sensor got initialized through this script.
2. bar30_readings.py  - From the pressure sensor we will estimate the depth values.
3. orientation_data.py - From the IMU sensor, we will get orientation of our robot.
4. dvl_publisher.py - From the DVL sensor, we will get the velocity measurements with the help of sound waves.

Sensors used in AUV3:
1. DVL - Doppler Velocity Log.
2. Pressure Sensor - For depth measurement.
3. Inertial Measurement Sensor (IMS) - Uses accelrometer, magnetometer and gyroscope to determine the orientation of the robot.
4. GPS - Used for getting the latitude and longtitude information.
