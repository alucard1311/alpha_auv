# AUV4
---
## 1. Directory Structure


`auv3_sim` containes all the necessary packages for simulation

`auv4_bringup` launch all the necessary nodes

`auv4_config` parameters required for mvp-helm 

`auv4_driver` sensor driver files

`auv4_localisation` yet to intergrate on the hardware

---

## Installation
---
### MVP Installation: in the ros workspace

```bash
git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_msgs
git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_control
git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/mvp_mission
git clone --single-branch --branch noetic-devel https://github.com/uri-ocean-robotics/stonefish_mvp
```

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
---

## Authors


Copyright (C) 2023 Autonomous and Undersea Subsystems Division

```bash
authors = {Vinay, Yash},
year    = {2023}
mail    = {vinay_atd@thesalemaeropark.com}

```
