cmake_minimum_required(VERSION 3.0.2)
project(emat)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

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

install(TARGETS emat_thickness_gauge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
