cmake_minimum_required(VERSION 3.0.2)
project(imu_wheel_localization)

find_package(Eigen3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  irp_sen_msgs
)

catkin_package(
  CATKIN_DEPENDS message_filters sensor_msgs geometry_msgs roscpp irp_sen_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(wheel_imu_kf src/wheel_imu_kf.cpp)
target_link_libraries(wheel_imu_kf ${catkin_LIBRARIES})

add_executable(wheel_imu_eskf src/wheel_imu_eskf.cpp)
target_link_libraries(wheel_imu_eskf ${catkin_LIBRARIES})

add_executable(wheel_imu_eskf2 src/wheel_imu_eskf2.cpp)
target_link_libraries(wheel_imu_eskf2 ${catkin_LIBRARIES})

add_executable(encoder src/encoder_process.cpp)
target_link_libraries(encoder ${catkin_LIBRARIES})