#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <deque>
#include <tf2/LinearMath/Quaternion.h>
// 状态结构体
struct State {
    Eigen::Vector3d position;  // 位置 [x, y, z]
    Eigen::Vector3d velocity;  // 速度 [vx, vy, vz]
    double theta;              // 偏航角
    Eigen::MatrixXd P;         // 状态协方差矩阵
    Eigen::MatrixXd Q;         // 过程噪声
    Eigen::MatrixXd R;         // 观测噪声

    State() {
        position.setZero();
        velocity.setZero();
        theta = 0.0;
        P = Eigen::MatrixXd::Identity(6, 6);
        Q = Eigen::MatrixXd::Identity(6, 6) * 0.001;
        R = Eigen::MatrixXd::Identity(2, 2) * 0.001;
    }
};

State state; // 全局状态

nav_msgs::Path path_msg;
ros::Publisher path_pub;
ros::Publisher odom_pub;
bool initialized = false;
ros::Time last_timestamp;
double last_update_time = 0.0; // 上次更新时间
Eigen::Vector3d gravity(0.0, 0.0, 9.81); // 重力加速度向量

std::deque<sensor_msgs::Imu::ConstPtr> imu_queue;
std::deque<geometry_msgs::TwistStamped::ConstPtr> wheel_odom_queue;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    if (msg) {
        imu_queue.push_back(msg);
    } else {
        ROS_WARN("Received null IMU message!");
    }
}

void wheelOdomCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (msg) {
        wheel_odom_queue.push_back(msg);
    } else {
        ROS_WARN("Received null wheel odom message!");
    }
}

// 发布路径 (Path)
void publishPath() {
    // 创建 PoseStamped 消息
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world"; // 或者根据您的坐标系进行调整

    pose_msg.pose.position.x = state.position(0);
    pose_msg.pose.position.y = state.position(1);
    pose_msg.pose.position.z = 0.0;

    // 使用偏航角设置姿态 (四元数)
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta); // roll, pitch, yaw
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // 添加到路径消息中
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world"; // 调整为您的坐标系
    path_msg.poses.push_back(pose_msg);

    // 控制路径长度，防止 rviz 崩溃
    static const size_t max_path_size = 1000000; // 设置路径点的最大数量
    if (path_msg.poses.size() > max_path_size) {
        path_msg.poses.erase(path_msg.poses.begin());
    }

    // 发布路径消息
    path_pub.publish(path_msg);

}

// 发布里程计 (Odometry)
void publishOdometry() {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";  // 或者根据您的坐标系进行调整
    odom_msg.child_frame_id = "base_link"; // 或者根据您的坐标系进行调整

    // 设置位置
    odom_msg.pose.pose.position.x = state.position(0);
    odom_msg.pose.pose.position.y = state.position(1);
    odom_msg.pose.pose.position.z = 0.0;

    // 使用偏航角设置姿态 (四元数)
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // 设置速度
    odom_msg.twist.twist.linear.x = state.velocity(0);
    odom_msg.twist.twist.linear.y = state.velocity(1);
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;

    // 发布里程计
    odom_pub.publish(odom_msg);
}

// 初始化系统
void initializeSystem() {
    if (!imu_queue.empty() && !wheel_odom_queue.empty()) {
        auto imu_msg = imu_queue.front();
        auto wheel_msg = wheel_odom_queue.front();

        state.position.setZero();
        state.velocity.setZero();

        state.velocity(0) = wheel_msg->twist.linear.x;
        state.P = Eigen::MatrixXd::Identity(6, 6) * 0.001;
        state.theta = 0.0;
        ROS_INFO_STREAM("Position: " << state.position.transpose());
        ROS_INFO_STREAM("Velocity: " << state.velocity.transpose());
        ROS_INFO_STREAM("Theta: " << state.theta);

        initialized = true;
        last_timestamp = imu_msg->header.stamp;
        ROS_INFO_STREAM("initialized!");
    }
}


// 预测步骤
void predictState(const sensor_msgs::Imu::ConstPtr &imu_msg ,double delta_t) {

    Eigen::Vector3d linear_acceleration(0.0, 0.0, 0.0);

        linear_acceleration << imu_msg->linear_acceleration.x,
                               imu_msg->linear_acceleration.y,
                               imu_msg->linear_acceleration.z;

        // 重力补偿
        Eigen::Quaterniond imu_quat(imu_msg->orientation.w, imu_msg->orientation.x,
                                    imu_msg->orientation.y, imu_msg->orientation.z);
        Eigen::Vector3d gravity_world = imu_quat.toRotationMatrix() * gravity;
        linear_acceleration -= gravity_world;

        // 更新偏航角
        state.theta += imu_msg->angular_velocity.z * delta_t;

        // 将加速度转换到全局坐标系
        Eigen::Vector2d acceleration_global;
        acceleration_global << linear_acceleration.x() * cos(state.theta) - linear_acceleration.y() * sin(state.theta),
                            linear_acceleration.x() * sin(state.theta) + linear_acceleration.y() * cos(state.theta);

        // 更新速度和位置
        state.velocity.head<2>() += acceleration_global * delta_t;
        state.position.head<2>() += state.velocity.head<2>() * delta_t;

        // 更新协方差矩阵
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        F(0, 3) = delta_t; // 位置对速度的影响
        F(1, 4) = delta_t; // 同上
        F(0, 2) = -state.velocity(0) * sin(state.theta) * delta_t - state.velocity(1) * cos(state.theta) * delta_t;
        F(1, 2) = state.velocity(0) * cos(state.theta) * delta_t - state.velocity(1) * sin(state.theta) * delta_t;

        state.P = F * state.P * F.transpose() + state.Q;

        ROS_INFO_STREAM("predict Position: " << state.position.transpose());
        ROS_INFO_STREAM("predict Velocity: " << state.velocity.transpose());
        ROS_INFO_STREAM("predict Theta: " << state.theta);

}

void updateObservation(const geometry_msgs::TwistStamped::ConstPtr &wheel_msg, double delta_t) {
    double v_measured = wheel_msg->twist.linear.x; // 观测速度
    double theta_measured = state.theta + wheel_msg->twist.angular.z * delta_t; // 当前偏航角

    // 观测向量 z
    Eigen::Vector2d z;
    z << v_measured, theta_measured;

    // 初始化观测矩阵 H
    Eigen::Matrix<double, 2, 6> H;
    H.setZero();

    // 初始化预测观测值 z_pred
    Eigen::Vector2d z_pred;
    z_pred << state.velocity.x() * cos(state.theta), state.theta;

    // 初始化观测残差 y
    Eigen::Vector2d y = z - z_pred;

    // 迭代误差卡尔曼滤波参数
    const int max_iter = 5; // 最大迭代次数
    const double epsilon = 1e-3; // 收敛阈值

    // 初始化状态误差变量
    Eigen::VectorXd delta_x(6);
    delta_x.setZero();
    Eigen::MatrixXd K;

    for (int iter = 0; iter < max_iter; ++iter) {
        // 更新观测矩阵 H
        H(0, 3) = cos(state.theta + delta_x(2)); // vx 对速度观测的偏导
        H(0, 2) = -state.velocity.x() * sin(state.theta + delta_x(2)); // theta 对速度观测的偏导
        H(1, 2) = 1; // theta 对偏航角观测的偏导

        // 更新预测观测值 z_pred
        z_pred(0) = (state.velocity.x() + delta_x(3)) * cos(state.theta + delta_x(2));
        z_pred(1) = state.theta + delta_x(2);

        // 更新观测残差 y
        y = z - z_pred;

        // 计算卡尔曼增益
        Eigen::Matrix2d S = H * state.P * H.transpose() + state.R;
        K = state.P * H.transpose() * S.inverse();

        // 更新状态误差
        Eigen::VectorXd delta_x_new = K * y;

        // 检查收敛性
        if ((delta_x_new - delta_x).norm() < epsilon) {
            delta_x = delta_x_new;
            break;
        }

        // 更新误差
        delta_x = delta_x_new;
    }

    // 最终状态更新
    state.position.x() += delta_x(0);
    state.position.y() += delta_x(1);
    state.theta += delta_x(2);
    state.velocity.x() += delta_x(3);
    state.velocity.y() += delta_x(4);

    // 更新协方差矩阵
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    state.P = (I - K * H) * state.P * (I - K * H).transpose() + K * state.R * K.transpose();

    // 日志输出
    ROS_INFO_STREAM("Observation delta_x: " << delta_x.transpose());
    ROS_INFO_STREAM("Observation Position: " << state.position.transpose());
    ROS_INFO_STREAM("Observation Velocity: " << state.velocity.transpose());
    ROS_INFO_STREAM("Observation Theta: " << state.theta);
}

   
// 同步数据处理
void syncData() {
    if (!initialized) {
        initializeSystem();
        return;
    }

    // 确保队列非空
    while (!imu_queue.empty() && !wheel_odom_queue.empty()) {
        auto imu_msg = imu_queue.front();
        auto wheel_msg = wheel_odom_queue.front();

        auto imu_time = imu_msg->header.stamp.toSec();
        auto wheel_time = wheel_msg->header.stamp.toSec();

        // 检查时间差是否在阈值内
        if (std::abs(imu_time - wheel_time) < 0.003) { // 时间同步精度调整为 3ms
            // 计算 delta_t
            double current_time = std::min(imu_time, wheel_time); // 当前处理时间
            double delta_t = (last_update_time == 0.0) ? 0.001 : (current_time - last_update_time);

            if (delta_t <= 0.0) {
                ROS_WARN("Non-positive delta_t encountered!");
                imu_queue.pop_front();
                wheel_odom_queue.pop_front();
                continue;
            }

            // 状态预测与观测更新
            predictState(imu_msg, delta_t);
            updateObservation(wheel_msg, delta_t);

            // 更新上次处理时间
            last_update_time = current_time;

            // 发布路径
            publishPath();

            // 清理队列
            imu_queue.pop_front();
            wheel_odom_queue.pop_front();
        } else if (imu_time < wheel_time - 0.003) {
            imu_queue.pop_front();
        } else {
            wheel_odom_queue.pop_front();
        }
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "esikf_odometry");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);
    ros::Subscriber wheel_odom_sub = nh.subscribe("/wheel_odom", 10, wheelOdomCallback);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 10);
    path_pub = nh.advertise<nav_msgs::Path>("odometry/path", 10);
    
    ros::Rate rate(300);
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [](const ros::TimerEvent&) { syncData(); });

    ros::spin();
        rate.sleep();
    return 0;
}
