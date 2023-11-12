cmake_minimum_required(VERSION 3.0.2)
project(auv3_gps_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base
  robot_localization
  tf
  roslib
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auv3_gps_navigation
#  CATKIN_DEPENDS actionlib geometry_msgs move_base robot_localization roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
  PROGRAMS 
  src/twistTOtwiststamp.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(gps_waypoint src/gps_waypoint.cpp)
target_link_libraries(gps_waypoint ${catkin_LIBRARIES})