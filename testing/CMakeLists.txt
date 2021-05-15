cmake_minimum_required(VERSION 2.8.3)
project(testing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslint
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_node 
  src/main_node.cpp
  src/testing/Duplicator.cpp
)

target_link_libraries(main_node ${catkin_LIBRARIES})

roslint_cpp(
  src/main_node.cpp
  src/testing/Duplicator.cpp
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(test_node_tests 
    tests/test_node_tests.cpp
    src/testing/Duplicator.cpp
  )
  target_link_libraries(test_node_tests ${catkin_LIBRARIES})
endif()