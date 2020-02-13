cmake_minimum_required(VERSION 2.8.3)
project(navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_ros
  roscpp
  rospy
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES csuro_mapping
#  CATKIN_DEPENDS geometry_msgs roscpp rospy tf
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
