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
#  LIBRARIES navigation_mapping
#  CATKIN_DEPENDS geometry_msgs roscpp rospy tf
#  DEPENDS system_lib
)


include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(navigate_to_wp_node src/navigate_to_wp.cpp)
add_dependencies(navigate_to_wp_node ${catkin_EXPORTED_TARGETS} )
target_link_libraries(navigate_to_wp_node ${catkin_LIBRARIES} )
