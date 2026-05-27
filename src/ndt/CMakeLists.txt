cmake_minimum_required(VERSION 2.8.3)
project(ndt)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(PYTHON_EXECUTABLE "/usr/bin/python3")

# --- 查找依赖包 ---
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(aruco REQUIRED)
set(aruco_INCLUDE_DIRS /usr/local/include)
set(aruco_LIBRARY_DIRS /usr/local/lib)
set(aruco_LIBRARIES /usr/local/lib/libaruco.so)
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
)

link_directories(
    ${aruco_LIBRARY_DIRS}
)

# --- 包含目录 ---
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${aruco_INCLUDE_DIRS}
)

# --- Catkin 包配置 ---
catkin_package(
    # CATKIN_DEPENDS 中也需要加入 cv_bridge
    CATKIN_DEPENDS roscpp rospy mavros sensor_msgs cv_bridge image_transport
    # LIBRARIES your_library # 如果你创建了库
)

# --- 编译可执行文件 ---
add_executable(record src/record.cpp)
target_link_libraries(record
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(aruco_pose src/aruco_pose.cpp)
target_link_libraries(aruco_pose
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${aruco_LIBRARIES}
)

catkin_install_python(PROGRAMS
        scripts/csrt_ros.py
        scripts/normal_ros.py
        scripts/abs_pos.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
