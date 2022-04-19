project(plansys2_tools)
cmake_minimum_required(VERSION 3.5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  qt_gui_cpp
)

find_package(Qt5Widgets REQUIRED)
set(CMAKE_CXX_STANDARD 17)

catkin_package(
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp qt_gui_cpp
  LIBRARIES rqt_plansys2_performers
)
catkin_python_setup()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## RQT Performers
set(rqt_plansys2_performers_SRCS
  src/rqt_plansys2_performers/RQTPerformers.cpp
)

set(rqt_plansys2_performers_HDRS
  include/rqt_plansys2_performers/RQTPerformers.hpp
)

set(rqt_plansys2_performers_UIS
  src/rqt_plansys2_performers/rqt_plansys2_performers.ui
)

set(rqt_plansys2_performers_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_cpp(rqt_plansys2_performers_MOCS ${rqt_plansys2_performers_HDRS})
qt5_wrap_ui(rqt_plansys2_performers_UIS_H ${rqt_plansys2_performers_UIS})

include_directories(
  ${rqt_plansys2_performers_INCLUDE_DIRECTORIES}
)

add_library(rqt_plansys2_performers SHARED
  ${rqt_plansys2_performers_SRCS}
  ${rqt_plansys2_performers_MOCS}
  ${rqt_plansys2_performers_UIS_H}
)

target_link_libraries(
  rqt_plansys2_performers
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(rqt_plansys2_performers)


install(TARGETS rqt_plansys2_performers
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES plugin_performers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/rqt_plansys2_performers
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


