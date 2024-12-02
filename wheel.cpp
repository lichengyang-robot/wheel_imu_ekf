#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <queue>
#include <iostream>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/LinearMath/Quaternion.h>
constexpr double GRAVITY = 9.81; // 重力加速度常量

// 状态结构体
struct State {
    Eigen::Vector3d position;  // 位置
    Eigen::Vector3d velocity;  // 速度
    double theta;              // 偏航角
    Eigen::MatrixXd P;         // 协方差矩阵
};

// 全局变量
State state_;
Eigen::MatrixXd Q_ = Eigen::MatrixXd::Identity(5, 5) * 0.001; // 过程噪声
Eigen::MatrixXd R_ = Eigen::MatrixXd::Identity(5, 5) * 0.001;   // 测量噪声

ros::Publisher path_pub;
ros::Publisher odom_pub;
double last_update_time = 0.0; // 上次更新时间

bool initialized =false;

// 计算旋转矩阵
Eigen::Matrix3d computeRotationMatrix(const Eigen::Vector3d &angular_velocity, double delta_t) {
    double angle = angular_velocity.norm() * delta_t;
    Eigen::Vector3d axis = angular_velocity.normalized();
    Eigen::AngleAxisd rotation_vector(angle, axis);
    return rotation_vector.toRotationMatrix();
}

// 角度归一化函数，将角度归一化到 [-pi, pi]
double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 状态预测
void predictState(const sensor_msgs::Imu::ConstPtr &imu_msg, double delta_t) {
    // 提取IMU角速度和加速度
    Eigen::Vector3d angular_velocity(
        imu_msg->angular_velocity.x,
        imu_msg->angular_velocity.y,
        imu_msg->angular_velocity.z);
    Eigen::Vector3d acceleration(
        imu_msg->linear_acceleration.x,
        imu_msg->linear_acceleration.y,
        imu_msg->linear_acceleration.z);

    // 计算旋转矩阵
    Eigen::Matrix3d R = computeRotationMatrix(angular_velocity, delta_t);

    // 将加速度从IMU坐标系转换到世界坐标系并校正重力
    Eigen::Vector3d gravity_imu(0.0, 0.0, -GRAVITY);
    Eigen::Vector3d acceleration_world = R * acceleration;
    Eigen::Vector3d corrected_acceleration = acceleration_world - gravity_imu;

    // 更新速度和位置
    state_.velocity += corrected_acceleration * delta_t;
    state_.position += state_.velocity * delta_t + 0.5 * corrected_acceleration * delta_t * delta_t;
    // ROS_INFO_STREAM("delta_t: " << delta_t);
    // ROS_INFO_STREAM("delta_v: " << (corrected_acceleration * delta_t).transpose());
    // ROS_INFO_STREAM("delta_x: " << (state_.velocity * delta_t + 0.5 * corrected_acceleration * delta_t * delta_t).transpose());
    // 更新偏航角 (假设角速度z轴对应偏航角变化)
    state_.theta += angular_velocity.z() * delta_t;
    // state_.theta = normalizeAngle(state_.theta); // 归一化到 [-pi, pi] 范围

    // 更新状态转移矩阵 F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
    F(0, 3) = delta_t;  // x 位置与 x 速度的关系
    F(1, 4) = delta_t;  // y 位置与 y 速度的关系

    // 更新协方差矩阵
    state_.P = F * state_.P * F.transpose() + Q_;
}

// 状态更新
void updateWheelOdom(const nav_msgs::Odometry::ConstPtr &wheel_msg) {
    // 提取pose信息
    double wheel_x = wheel_msg->pose.pose.position.x;
    double wheel_y = wheel_msg->pose.pose.position.y;
    Eigen::Quaterniond q(
        wheel_msg->pose.pose.orientation.w,
        wheel_msg->pose.pose.orientation.x,
        wheel_msg->pose.pose.orientation.y,
        wheel_msg->pose.pose.orientation.z);
    double wheel_theta = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    // 提取twist信息
    double wheel_linear_x = wheel_msg->twist.twist.linear.x;
    double wheel_linear_y = wheel_msg->twist.twist.linear.y;

    // 构造观测向量z
    Eigen::VectorXd z(5);
    z << wheel_x,            // x 位置
         wheel_y,            // y 位置
         wheel_theta,        // 偏航角
         wheel_linear_x,     // 线速度
         wheel_linear_y;    // 角速度

    // 构造状态向量x
    Eigen::VectorXd x(5);
    x << state_.position(0),  // x 位置
         state_.position(1),  // y 位置
         state_.theta,        // 偏航角
         state_.velocity(0),  // x 轴速度
         state_.velocity(1);  // y 轴速度
    // ROS_INFO_STREAM("x: " << x.transpose());
    // ROS_INFO_STREAM("z: " << z.transpose());
    // 构造观测矩阵H
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(5, 5);
    H(0, 0) = 1.0;  // x 位置
    H(1, 1) = 1.0;  // y 位置
    H(2, 2) = 1.0;  // 偏航角
    H(3, 3) = 1.0;  // x 轴速度
    H(4, 4) = 1.0;  // 角速度

    // 计算残差y
    Eigen::VectorXd y = z - H * x;
    // ROS_INFO_STREAM("y: " << y.transpose());
    // 归一化偏航角差值到[-pi, pi]
    // y(2) = normalizeAngle(y(2));

    // 计算协方差矩阵S
    Eigen::MatrixXd S = H * state_.P * H.transpose() + R_;   //5x5

    // 计算卡尔曼增益K
    Eigen::MatrixXd K = state_.P * H.transpose() * S.inverse();  //5x5

    // 更新状态向量
    Eigen::VectorXd x_new = x + K * y; //5x1
    // ROS_INFO_STREAM("x_new: " << x_new.transpose());
    // 更新位置、偏航角和速度
    state_.position << x_new(0), x_new(1), 0.0;
    // state_.theta = normalizeAngle(x_new(2));
    state_.velocity << x_new(3), x_new(4), 0.0;

    // 更新协方差矩阵
    state_.P = (Eigen::MatrixXd::Identity(5, 5) - K * H) * state_.P;
}

// IMU 数据队列
std::queue<sensor_msgs::Imu::ConstPtr> imu_queue;
// 轮速计数据队列
std::queue<nav_msgs::Odometry::ConstPtr> wheel_queue;

// IMU 数据回调
void imuCallback(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    imu_queue.push(imu_msg);
}

// 轮速计数据回调
void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
    wheel_queue.push(odom_msg);
    // std::cout << "wheel_queue.size()=" << wheel_queue.size() << std::endl;
}


// 发布路径 (Path)
void publishPath() {
    // 创建 PoseStamped 消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world"; // 或者根据您的坐标系进行调整

    pose_msg.pose.position.x = state_.position(0);
    pose_msg.pose.position.y = state_.position(1);
    pose_msg.pose.position.z = 0.0;

    // 使用偏航角设置姿态 (四元数)
    tf2::Quaternion q;
    q.setRPY(0, 0, state_.theta); // roll, pitch, yaw
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) // if path is too large, the rvis will crash
    {
        // 发布到路径
        path_pub.publish(pose_msg);
    }

}

// 发布里程计 (Odometry)
void publishOdometry() {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";  // 或者根据您的坐标系进行调整
    odom_msg.child_frame_id = "base_link"; // 或者根据您的坐标系进行调整

    // 设置位置
    odom_msg.pose.pose.position.x = state_.position(0);
    odom_msg.pose.pose.position.y = state_.position(1);
    odom_msg.pose.pose.position.z = 0.0;

    // 使用偏航角设置姿态 (四元数)
    tf2::Quaternion q;
    q.setRPY(0, 0, state_.theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 设置速度
    odom_msg.twist.twist.linear.x = state_.velocity(0);
    odom_msg.twist.twist.linear.y = state_.velocity(1);
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // 发布里程计
    odom_pub.publish(odom_msg);
}

// 同步回调函数
void syncData() {
    if (!imu_queue.empty() && !wheel_queue.empty()) {
        sensor_msgs::Imu::ConstPtr imu_msg = imu_queue.front();
        nav_msgs::Odometry::ConstPtr wheel_msg = wheel_queue.front();

        double imu_time = imu_msg->header.stamp.toSec();
        double wheel_time = wheel_msg->header.stamp.toSec();
        if(!initialized){
            state_.position(0) = wheel_msg->pose.pose.position.x;
            state_.position(1) = wheel_msg->pose.pose.position.y;
            state_.position(2) = wheel_msg->pose.pose.position.z;
            state_.velocity(0) = imu_msg->angular_velocity.x;
            state_.velocity(1) = imu_msg->angular_velocity.y;
            state_.velocity(2) = imu_msg->angular_velocity.z;
            state_.theta = 0.0;
            state_.P = Eigen::MatrixXd::Identity(5, 5) * 0.001;
            ROS_INFO("initialized!");
            ROS_INFO_STREAM("Position: " << state_.position.transpose());
            ROS_INFO_STREAM("Velocity: " << state_.velocity.transpose());
            ROS_INFO_STREAM("Theta: " << state_.theta);
            initialized = true;
        }
        // 时间戳差值小于阈值，认为数据同步
        if (std::abs(imu_time - wheel_time) < 0.01) {
            double delta_t = (last_update_time == 0.0) ? 0.01 : (imu_time - last_update_time);
            if (delta_t <= 0.0) {
                ROS_WARN("Non-positive delta_t encountered!");
                return;
            }

            // 状态预测与更新
            predictState(imu_msg, delta_t);
            updateWheelOdom(wheel_msg);

            // 更新时间戳
            last_update_time = imu_time;

            // 发布状态的路径和里程计
            publishPath();
            publishOdometry();

            // 输出状态
            ROS_INFO_STREAM("Position: " << state_.position.transpose());
            ROS_INFO_STREAM("Velocity: " << state_.velocity.transpose());
            ROS_INFO_STREAM("Theta: " << state_.theta);

            // 数据处理后移除
            imu_queue.pop();
            wheel_queue.pop();
        } else if (imu_time < wheel_time) {
            imu_queue.pop();  // IMU 数据时间戳较小，丢弃
        } else {
            wheel_queue.pop(); // 轮速计数据时间戳较小，丢弃
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_wheel_fusion_node");
    ros::NodeHandle nh;

    // 初始化状态
    state_.position = Eigen::Vector3d::Zero();
    state_.velocity = Eigen::Vector3d::Zero();
    state_.theta = 0.0;
    state_.P = Eigen::MatrixXd::Identity(5, 5) * 0.001;

    // 订阅 IMU 和轮速计话题
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 100, imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/ridgeback_velocity_controller/odom", 100, odomCallback);

    // 发布 Path 和 Odometry
    path_pub = nh.advertise<nav_msgs::Path>("/robot_pat", 100);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/robot_odometry", 100);
    // 定时器用于调用同步数据处理函数
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [](const ros::TimerEvent&) { syncData(); });

    // 启动回调处理
    ros::spin();

    return 0;
}
