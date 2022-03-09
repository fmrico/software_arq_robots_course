cmake_minimum_required(VERSION 2.8.3)
project(behavior_trees)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  behaviortree_cpp_v3
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs kobuki_msgs sensor_msgs behaviortree_cpp_v3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(nodo_behavior_trees 
  src/nodo_behavior_trees.cpp
  src/behavior_trees/ApproachObject.cpp
  src/behavior_trees/CheckBattery.cpp
  src/behavior_trees/OpenGripper.cpp
  src/behavior_trees/CloseGripper.cpp
)
target_link_libraries(nodo_behavior_trees ${catkin_LIBRARIES})

add_executable(nodo_bump 
  src/nodo_bump.cpp
  src/behavior_trees/ApproachObject.cpp
  src/behavior_trees/CheckBattery.cpp
  src/behavior_trees/OpenGripper.cpp
  src/behavior_trees/CloseGripper.cpp
  src/behavior_trees/Back.cpp
  src/behavior_trees/Is_bumped.cpp
  src/behavior_trees/Forward.cpp
  src/behavior_trees/Turn.cpp
)
target_link_libraries(nodo_bump ${catkin_LIBRARIES})


roslint_cpp(
  src/nodo_behavior_trees.cpp
  include/behavior_trees/ApproachObject.h src/behavior_trees/ApproachObject.cpp
  include/behavior_trees/CheckBattery.h src/behavior_trees/CheckBattery.cpp
  include/behavior_trees/OpenGripper.h src/behavior_trees/OpenGripper.cpp
  include/behavior_trees/CloseGripper.h src/behavior_trees/CloseGripper.cpp
)
