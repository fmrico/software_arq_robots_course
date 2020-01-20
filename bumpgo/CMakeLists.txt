cmake_minimum_required(VERSION 2.8.3)
project(bumpgo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nodo_bumpgo src/nodo_bumpgo.cpp)

target_link_libraries(nodo_bumpgo ${catkin_LIBRARIES})


roslint_cpp(
 src/nodo_bumpgo.cpp
)
