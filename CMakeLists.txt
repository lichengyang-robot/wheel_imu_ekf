cmake_minimum_required(VERSION 3.0.2)
project(imu_wheel_localization)

find_package(Eigen3)

## 找到catkin所需的包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  irp_sen_msgs
)

## 声明catkin包
catkin_package(
  CATKIN_DEPENDS message_filters sensor_msgs geometry_msgs roscpp irp_sen_msgs
)

## 包含头文件目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 编译节点
add_executable(kf_node wheel_kf.cpp)
target_link_libraries(kf_node ${catkin_LIBRARIES})

## 编译节点
add_executable(eskf_node wheel_eskf.cpp)
target_link_libraries(eskf_node ${catkin_LIBRARIES})

## 编译节点
add_executable(encoder encoder_process.cpp)
target_link_libraries(encoder ${catkin_LIBRARIES})