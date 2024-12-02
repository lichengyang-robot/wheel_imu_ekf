cmake_minimum_required(VERSION 3.0.2)
project(imu_wheel_localization)

find_package(Eigen3)

## 找到catkin所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS message_filters sensor_msgs geometry_msgs roscpp
)

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 编译节点
add_executable(ekf_node wheel.cpp)
target_link_libraries(ekf_node ${catkin_LIBRARIES})
