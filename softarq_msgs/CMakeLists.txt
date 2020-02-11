cmake_minimum_required(VERSION 2.8.3)
project(softarq_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  actionlib
  actionlib_msgs
)

add_message_files(
   DIRECTORY msg
   FILES Point.msg
 )

add_service_files(
   DIRECTORY srv
   FILES Distance.srv
)

add_action_files(
   DIRECTORY action
   FILES Sequence.action
)

generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime actionlib_msgs
)
