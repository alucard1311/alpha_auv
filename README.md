# ALPHA AUV
---
## 1. Directory Structure


`auv3_sim` containes all the necessary packages for simulation

`auv4_bringup` launch all the necessary nodes

`auv4_config` parameters required for mvp-helm 

`auv4_driver` sensor driver files

`auv4_localisation` yet to intergrate on the hardware

---

## Installation


## Test The Simulation
---
### to launch the simulated environment
```bash
roslaunch auv4_bringup auv4_bringup.launch
```

### to launch the mvp node
```bash
roslaunch auv4_bringup auv4_helm.launch
```

### available States to use
```bash
survey_local # to start the local navigation
start # to make the auv idle mode and hold
kill # to kill the mission
```
### To Start the navigation
```bash
#Enable Serivices
rosservice call /auv3/controller/enable
rosservice call /auv3/helm/change_state "state: 'survey_global'"
```
```bash
#run navigation controller
roscd auv4_driver/src
python3 navigation_controller.py
```

### To edit the waypoints for gps_navigation
```bash
roscd auv4_config/param/gps_wpt.yaml
```
