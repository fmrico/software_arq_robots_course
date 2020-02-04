cmake_minimum_required(VERSION 2.8.3)
project(geometry_tf)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  tf2_geometry_msgs
  roscpp
  tf2
  tf2_ros
  roslint
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES geometry_tf
   CATKIN_DEPENDS geometry_msgs tf2_geometry_msgs roscpp tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(geometry_tf src/geometry_tf/transforms.cpp)

set(LINK_LIBS
  geometry_tf
  ${catkin_LIBRARIES}
)

add_executable(nodo_tf_pub 
	src/nodo_tf_pub.cpp
)

add_executable(nodo_tf_sub 
	src/nodo_tf_sub.cpp
)

target_link_libraries(nodo_tf_pub ${LINK_LIBS})
target_link_libraries(nodo_tf_sub ${LINK_LIBS})

roslint_cpp(
  src/geometry_tf/transforms.cpp include/geometry_tf/transforms.h
  src/nodo_tf_sub.cpp
  src/nodo_tf_pub.cpp
)