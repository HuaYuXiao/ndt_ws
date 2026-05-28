cmake_minimum_required(VERSION 2.8.3)
project(ndt)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_AUTOMOC ON)

set(PYTHON_EXECUTABLE "/usr/bin/python3")

# --- 查找依赖包 ---
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        mavros
        tf
        tf2_ros
        tf2_eigen
        message_filters
        tf2_geometry_msgs
        mavros_msgs
        geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
        cv_bridge
        image_transport
        rviz
        pluginlib
)

# --- 包含目录 ---
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

# --- Catkin 包配置 ---
catkin_package(
    CATKIN_DEPENDS roscpp rospy mavros sensor_msgs cv_bridge image_transport
)

# --- 编译可执行文件 ---
add_executable(record src/record.cpp)
target_link_libraries(record
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# --- RViz target selection panel plugin ---
set(PANEL_SOURCES
    src/rviz_target_panel.cpp
    include/ndt/rviz_target_panel.h
)

add_library(rviz_target_panel ${PANEL_SOURCES})

target_include_directories(rviz_target_panel PRIVATE
    ${Qt5Widgets_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
)

target_link_libraries(rviz_target_panel
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    Qt5::Widgets
)

add_dependencies(rviz_target_panel
    ${catkin_EXPORTED_TARGETS}
)

# --- Install ---
install(TARGETS record
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rviz_target_panel
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
        scripts/csrt_ros.py
        scripts/normal_ros.py
        scripts/abs_pos.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
