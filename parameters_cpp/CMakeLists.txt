cmake_minimum_required(VERSION 2.8.3)
project(parameters_cpp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  CATKIN_DEPENDS roscpp 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(nodo_parameter_reader src/nodo_parameter_reader.cpp)
target_link_libraries(nodo_parameter_reader ${catkin_LIBRARIES})
