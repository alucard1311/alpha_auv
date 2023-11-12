cmake_minimum_required(VERSION 3.0.2)
project(auv3_description)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
