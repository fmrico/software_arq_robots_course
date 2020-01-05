cmake_minimum_required(VERSION 2.8.3)
project(basics_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nodo_simple src/nodo_simple.cpp)
add_executable(nodo_iterativo src/nodo_iterativo.cpp)
add_executable(nodo_sub src/nodo_sub.cpp)
add_executable(nodo_pub src/nodo_pub.cpp)
add_executable(nodo_sub_obj src/nodo_sub_obj.cpp)
add_executable(nodo_sub_async src/nodo_sub_async.cpp)

target_link_libraries(nodo_simple ${catkin_LIBRARIES})
target_link_libraries(nodo_iterativo ${catkin_LIBRARIES})
target_link_libraries(nodo_sub ${catkin_LIBRARIES})
target_link_libraries(nodo_pub ${catkin_LIBRARIES})
target_link_libraries(nodo_sub_obj ${catkin_LIBRARIES})
target_link_libraries(nodo_sub_async ${catkin_LIBRARIES})

roslint_cpp(
  src/nodo_simple.cpp
  src/nodo_iterativo.cpp
  src/nodo_sub.cpp
  src/nodo_sub_obj.cpp
  src/nodo_sub_async.cpp
)
