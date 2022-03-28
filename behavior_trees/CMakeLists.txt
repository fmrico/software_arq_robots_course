cmake_minimum_required(VERSION 2.8.3)
project(behavior_trees)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  actionlib
  actionlib_msgs
  move_base_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs behaviortree_cpp_v3 actionlib_msgs move_base_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
)

add_library(asr_move_bt_node SHARED src/behavior_trees/Move.cpp)
add_library(asr_approach_object_bt_node SHARED src/behavior_trees/ApproachObject.cpp)
add_library(asr_check_battery_bt_node SHARED src/behavior_trees/CheckBattery.cpp)
add_library(asr_close_gripper_bt_node SHARED src/behavior_trees/CloseGripper.cpp)
add_library(asr_open_gripper_bt_node SHARED src/behavior_trees/OpenGripper.cpp)
list(APPEND plugin_libs
  asr_move_bt_node
  asr_approach_object_bt_node
  asr_check_battery_bt_node
  asr_close_gripper_bt_node
  asr_open_gripper_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(nodo_behavior_trees 
  src/nodo_behavior_trees.cpp
)
target_link_libraries(nodo_behavior_trees ${catkin_LIBRARIES} ${ZMQ_LIBRARIES})


roslint_cpp(
  src/nodo_behavior_trees.cpp
  include/behavior_trees/ApproachObject.h src/behavior_trees/ApproachObject.cpp
  include/behavior_trees/CheckBattery.h src/behavior_trees/CheckBattery.cpp
  include/behavior_trees/OpenGripper.h src/behavior_trees/OpenGripper.cpp
  include/behavior_trees/CloseGripper.h src/behavior_trees/CloseGripper.cpp
)
