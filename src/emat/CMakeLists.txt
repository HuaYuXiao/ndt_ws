cmake_minimum_required(VERSION 3.0.2)
project(emat)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rviz)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_message_files(FILES EmatWaveform.msg EmatThickness.msg EmatDeviceStatus.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS roscpp std_msgs message_runtime)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_executable(emat_thickness_gauge_node
  src/emat_thickness_gauge_node.cpp
)

target_link_libraries(emat_thickness_gauge_node
  ${catkin_LIBRARIES}
  ${LIBUSB_LIBRARIES}
  pthread
)

add_dependencies(emat_thickness_gauge_node
  ${catkin_EXPORTED_TARGETS}
)

# RViz panel plugin
set(PANEL_SOURCES
  src/rviz_emat_panel.cpp
  src/waveform_widget.cpp
  include/emat_thickness_gauge/rviz_emat_panel.h
  include/emat_thickness_gauge/waveform_widget.h
)

add_library(rviz_emat_panel ${PANEL_SOURCES})

target_include_directories(rviz_emat_panel PRIVATE
  ${Qt5Widgets_INCLUDE_DIRS}
)

target_link_libraries(rviz_emat_panel
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

add_dependencies(rviz_emat_panel
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS emat_thickness_gauge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS rviz_emat_panel
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
