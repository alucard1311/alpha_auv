# AUV3 GPS based Navigation

## Steps
1. Launch Simulation, Localization and Rviz
```bash
roslaunch auv3_localization combined.launch
```
2. Launch move_base
```bash
roslaunch auv3_gps_navigation move_base2.launch
```
3. Send GPS Waypoint
```bash
roslaunch auv3_gps_navigation gps_navigation.launch
```

## Dependencies
* robot_localization
* tf
* actionlib
* Dave sensors repository