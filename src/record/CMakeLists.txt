cmake_minimum_required(VERSION 2.8.3)
project(record)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_AUTOMOC ON)

set(PYTHON_EXECUTABLE "/usr/bin/python3")

find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslib
    std_msgs
    nav_msgs
    sensor_msgs
    mavros_msgs
    geometry_msgs
    cv_bridge
    tf2
    tf2_ros
    rviz
    pluginlib
    emat
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS roscpp roslib std_msgs nav_msgs sensor_msgs mavros_msgs geometry_msgs cv_bridge tf2 tf2_ros rviz pluginlib emat
)

# --- Multi-modal recorder ---
add_executable(multimodal_recorder src/multimodal_recorder.cpp)
target_link_libraries(multimodal_recorder
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(multimodal_recorder
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

# --- Install ---
install(TARGETS multimodal_recorder
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
    scripts/rosbag_to_dataset.py
    scripts/label_tool.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# --- RViz record control panel plugin ---
set(PANEL_SOURCES
    src/rviz_record_panel.cpp
    include/record/rviz_record_panel.h
)
add_library(rviz_record_panel ${PANEL_SOURCES})
target_include_directories(rviz_record_panel PRIVATE
    ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(rviz_record_panel
    ${catkin_LIBRARIES}
    Qt5::Widgets
)
add_dependencies(rviz_record_panel ${catkin_EXPORTED_TARGETS})

install(TARGETS rviz_record_panel
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
