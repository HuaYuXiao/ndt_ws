cmake_minimum_required(VERSION 2.8.3)
project(bringup)

ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros
  roscpp
  rospy
  std_msgs
  nav_msgs
)
find_package(PCL 1.8 REQUIRED)

# --- Catkin 包配置 ---
catkin_package(
    # CATKIN_DEPENDS 中也需要加入 cv_bridge
    CATKIN_DEPENDS roscpp rospy geometry_msgs nav_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
        scripts/lidar_to_mavros.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
