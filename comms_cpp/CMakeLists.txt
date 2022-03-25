cmake_minimum_required(VERSION 2.8.3)
project(comms_cpp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  softarq_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs actionlib_msgs softarq_msgs move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nodo_point_sub src/nodo_point_sub.cpp)
add_executable(nodo_point_pub src/nodo_point_pub.cpp)
add_executable(nodo_distance_server src/nodo_distance_server.cpp)
add_executable(nodo_distance_client src/nodo_distance_client.cpp)
add_executable(nodo_sequence_server src/nodo_sequence_server.cpp)
add_executable(nodo_sequence_client src/nodo_sequence_client.cpp)
add_executable(nodo_nav_client src/nodo_nav_client.cpp)

target_link_libraries(nodo_point_sub ${catkin_LIBRARIES})
target_link_libraries(nodo_point_pub ${catkin_LIBRARIES})
target_link_libraries(nodo_distance_client ${catkin_LIBRARIES})
target_link_libraries(nodo_distance_server ${catkin_LIBRARIES})
target_link_libraries(nodo_sequence_server ${catkin_LIBRARIES})
target_link_libraries(nodo_sequence_client ${catkin_LIBRARIES})
target_link_libraries(nodo_nav_client ${catkin_LIBRARIES})

add_dependencies(nodo_point_sub softarq_msgs_generate_messages_cpp)
add_dependencies(nodo_point_pub softarq_msgs_generate_messages_cpp)
add_dependencies(nodo_distance_client softarq_msgs_generate_messages_cpp)
add_dependencies(nodo_distance_server softarq_msgs_generate_messages_cpp)
add_dependencies(nodo_sequence_server softarq_msgs_generate_messages_cpp)
add_dependencies(nodo_sequence_client softarq_msgs_generate_messages_cpp)
