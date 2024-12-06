#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <deque>

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
        Q = Eigen::MatrixXd::Identity(6, 6) * 0.01;
        R = Eigen::MatrixXd::Identity(2, 2) * 0.1;
    }
};

State state; // 全局状态

ros::Publisher state_pub;
ros::Publisher path_pub;
nav_msgs::Path path_msg;

bool initialized = false;
ros::Time last_timestamp;

Eigen::Vector3d gravity(0.0, 0.0, 9.81); // 重力加速度向量

std::deque<sensor_msgs::Imu> imu_queue;
std::deque<nav_msgs::Odometry> wheel_odom_queue;

// IMU 回调
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    imu_queue.push_back(*msg);
    if (imu_queue.size() > 100) imu_queue.pop_front();
}

// 轮速回调
void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    wheel_odom_queue.push_back(*msg);
    if (wheel_odom_queue.size() > 100) wheel_odom_queue.pop_front();
}

// 初始化系统
void initializeSystem() {
    if (!imu_queue.empty() && !wheel_odom_queue.empty()) {
        auto imu_msg = imu_queue.front();
        auto wheel_msg = wheel_odom_queue.front();

        state.position.setZero();

        // 初始化偏航角
        Eigen::Quaterniond quat(wheel_msg.pose.pose.orientation.w,
                                wheel_msg.pose.pose.orientation.x,
                                wheel_msg.pose.pose.orientation.y,
                                wheel_msg.pose.pose.orientation.z);
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
        state.theta = euler(2);

        // 初始化速度
        state.velocity(0) = wheel_msg.twist.twist.linear.x;

        ROS_INFO_STREAM("Position: " << state.position.transpose());
        ROS_INFO_STREAM("Velocity: " << state.velocity.transpose());
        ROS_INFO_STREAM("Theta: " << state.theta);

        initialized = true;
        last_timestamp = imu_msg.header.stamp;
    }
}

// 预测步骤
void predictState(const ros::Time& timestamp) {
    double delta_t = (timestamp - last_timestamp).toSec();
    if (delta_t <= 0.0) return;

    Eigen::Vector3d linear_acceleration(0.0, 0.0, 0.0);
    double angular_velocity_z = 0.0;

    if (!imu_queue.empty()) {
        auto imu_msg = imu_queue.front();
        linear_acceleration << imu_msg.linear_acceleration.x,
                               imu_msg.linear_acceleration.y,
                               imu_msg.linear_acceleration.z;

        // 重力补偿
        Eigen::Quaterniond imu_quat(imu_msg.orientation.w, imu_msg.orientation.x,
                                    imu_msg.orientation.y, imu_msg.orientation.z);
        Eigen::Vector3d gravity_world = imu_quat.toRotationMatrix() * gravity;
        linear_acceleration -= gravity_world;

        angular_velocity_z = imu_msg.angular_velocity.z;
        imu_queue.pop_front();
    }

    // 更新偏航角
    state.theta += angular_velocity_z * delta_t;

    // 将加速度转换到全局坐标系
    Eigen::Vector2d acceleration_global;
    acceleration_global << linear_acceleration.x() * cos(state.theta) - linear_acceleration.y() * sin(state.theta),
                           linear_acceleration.x() * sin(state.theta) + linear_acceleration.y() * cos(state.theta);

    // 更新速度和位置
    state.velocity.head<2>() += acceleration_global * delta_t;
    state.position.head<2>() += state.velocity.head<2>() * delta_t;

    // 更新协方差矩阵
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F(0, 4) = delta_t; // 位置对速度的影响
    F(1, 5) = delta_t; // 同上

    state.P = F * state.P * F.transpose() + state.Q;
    last_timestamp = timestamp;
}

void updateObservation() {
    if (!initialized) return;
    if (!wheel_odom_queue.empty()) {
        auto wheel_msg = wheel_odom_queue.front();
        // 轮速计观测：测量速度和偏航角
        double v_measured = wheel_msg.twist.twist.linear.x;
        double theta_measured = state.theta; // 当前状态中的偏航角

        // 修正轮速的速度（全局坐标系投影）
        double corrected_v = v_measured * cos(state.theta);

        // 观测向量 z
        Eigen::Vector2d z;
        z << corrected_v, theta_measured;

        // 观测矩阵 H
        Eigen::Matrix<double, 2, 4> H;
        H << 0, 0, 0, 1,  // 观测速度
            0, 0, 1, 0;  // 观测偏航角

        // 预测的观测值
        Eigen::Vector2d z_pred;
        z_pred << state.velocity(0), state.theta;

        // 观测残差
        Eigen::Vector2d y = z - z_pred;

        // 卡尔曼增益
        Eigen::Matrix<double, 4, 2> K = state.P * H.transpose() * (H * state.P * H.transpose() + state.R).inverse();

        // 迭代更新：使用 ESIKF 更新误差状态
        int max_iter = 5; // 设置最大迭代次数
        for (int i = 0; i < max_iter; ++i) {
            // 更新误差状态
            Eigen::Vector4d delta_x = K * y;
            state += delta_x;

            // 更新协方差矩阵
            state.P = (Eigen::Matrix4d::Identity() - K * H) * state.P;

            // 重新计算预测观测值
            z_pred = state.head<2>(); // [速度, 偏航角]
            y = z - z_pred; // 更新残差

            // 重新计算卡尔曼增益
            K = state.P * H.transpose() * (H * state.P * H.transpose() + state.R).inverse();
        }
    }

}
// 同步数据处理
void syncData() {
    if (!initialized) initializeSystem();
    if (!initialized) return;

    while (!imu_queue.empty() && !wheel_odom_queue.empty()) {
        auto imu_time = imu_queue.front().header.stamp.toSec();
        auto wheel_time = wheel_odom_queue.front().header.stamp.toSec();

        if (std::abs(imu_time - wheel_time) < 0.01) {
            ros::Time sync_time = imu_queue.front().header.stamp;
            predictState(sync_time);
            ROS_INFO_STREAM("Position: " << state.position.transpose());
            ROS_INFO_STREAM("Velocity: " << state.velocity.transpose());
            ROS_INFO_STREAM("Theta: " << state.theta);
            updateObservation();
            imu_queue.pop_front();
            wheel_odom_queue.pop_front();
        } else if (imu_time < wheel_time) {
            imu_queue.pop_front();
        } else {
            wheel_odom_queue.pop_front();
        }
    }
}

// 发布里程计消息
void publishOdometry(const ros::Time& timestamp) {
    nav_msgs::Odometry corrected_state;
    corrected_state.header.stamp = timestamp;
    corrected_state.header.frame_id = "world";
    corrected_state.child_frame_id = "base_link"; // 或者根据您的坐标系进行调整
    corrected_state.pose.pose.position.x = state(0);
    corrected_state.pose.pose.position.y = state(1);
    corrected_state.pose.pose.position.z = 0.0;

    // 从偏航角构造四元数
    Eigen::Quaterniond corrected_quat(Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()));
    corrected_state.pose.pose.orientation.w = corrected_quat.w();
    corrected_state.pose.pose.orientation.x = corrected_quat.x();
    corrected_state.pose.pose.orientation.y = corrected_quat.y();
    corrected_state.pose.pose.orientation.z = corrected_quat.z();

    corrected_state.twist.twist.linear.x = state(3);

    state_pub.publish(corrected_state);
}

// 更新路径并发布
void updateAndPublishPath(const ros::Time& timestamp) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = timestamp;
    pose.header.frame_id = "world";
    pose.pose.position.x = state(0);
    pose.pose.position.y = state(1);
    pose.pose.position.z = 0.0;

    // 从偏航角构造四元数
    Eigen::Quaterniond corrected_quat(Eigen::AngleAxisd(state(2), Eigen::Vector3d::UnitZ()));
    pose.pose.orientation.w = corrected_quat.w();
    pose.pose.orientation.x = corrected_quat.x();
    pose.pose.orientation.y = corrected_quat.y();
    pose.pose.orientation.z = corrected_quat.z();

    path_msg.poses.push_back(pose);
    path_pub.publish(path_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "esikf_odometry");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("imu/data", 10, imuCallback);
    ros::Subscriber wheel_odom_sub = nh.subscribe("/ridgeback_velocity_controller/odom", 10, wheelOdomCallback);
    state_pub = nh.advertise<nav_msgs::Odometry>("odometry/filtered", 10);
    path_pub = nh.advertise<nav_msgs::Path>("odometry/path", 10);

    ros::Rate rate(20);
    ros::Timer timer = nh.createTimer(ros::Duration(0.01), [](const ros::TimerEvent&) { syncData(); });

    ros::spin();
    return 0;
}
