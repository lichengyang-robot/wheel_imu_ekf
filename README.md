# Wheel_IMU_EKF

This repository implements **Extended Kalman Filter (EKF)** to fuse wheel odometry and IMU data for robust motion estimation.

---
![alt text](pics/image-1.png)
![alt text](pics/image.png)
## Features

- Real-time fusion of wheel odometry and IMU data.
- Noise filtering and state estimation with an EKF.
- Designed for applications in robotics and autonomous vehicles.

## Data Source

The dataset and reference implementation are inspired by:

```bibtex
@misc{ztd2021viwo,
  title={VIW-Fusion: Visual-Inertial-Wheel Fusion Odometry},
  author={Tingda Zhuang},
  howpublished={\url{https://github.com/TouchDeeper/VIW-Fusion}},
  year={2021}
}
```

## Requirements

To use this repository, ensure you have the following dependencies installed:

- **ROS** (Robot Operating System) - For data handling and communication.
- **Eigen3** - Linear algebra library.
- **C++11 or higher** - Required for modern C++ features.

## Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/lichengyang-robot/wheel_imu_ekf.git
   cd wheel_imu_ekf
   ```

2. Build the workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   mv wheel_imu_ekf ~/catkin_ws/src/
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. Run the nodes:
   ```bash
   roslaunch wheel_imu_ekf ekf_fusion.launch
   ```

## Usage

- **Input Topics:**
  - `/imu/data_raw`: Raw IMU data (linear acceleration, angular velocity).
  - `/wheel/odometry`: Wheel odometry data.

- **Output Topics:**
  - `/fusion/pose`: Fused pose (position and orientation).
  - `/fusion/path`: Fused trajectory.

- **Configuration:**
  Modify parameters in `config/ekf_params.yaml` to tune the filter for your application.

## Examples

1. Visualize the fused trajectory in `rviz`:
   ```bash
   rosrun rviz rviz
   ```

2. Test with a sample bag file:
   ```bash
   rosbag play data/sample.bag
   ```

## Contribution

Contributions are welcome! If you have any suggestions or improvements, feel free to submit a pull request or open an issue.
