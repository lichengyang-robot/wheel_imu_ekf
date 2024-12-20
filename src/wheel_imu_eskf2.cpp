#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <deque>
#include <mutex>
#include <tf/transform_datatypes.h>
#include <tf/tf.h> // 必须包含的头文件
#include <tf_conversions/tf_eigen.h> 
#include <tf2_eigen/tf2_eigen.h>
#include <cmath> // 确保包含 math 头文件

#include <irp_sen_msgs/encoder.h>
#include <irp_sen_msgs/imu.h>
constexpr double GRAVITY = 9.81; // 重力加速度常量
// === 全局变量和数据结构 ===
// 状态结构体
struct State {
    Eigen::Vector2d position;  // 位置 [x, y]
    Eigen::Vector2d velocity;  // 速度 [vx, vy] 线速度,角速度
    double theta;              // 偏航角
    Eigen::MatrixXd P;         // 状态协方差矩阵
    Eigen::MatrixXd Q;         // 过程噪声
    Eigen::MatrixXd R;         // 观测噪声

    State() {
        position.setZero();
        velocity.setZero();
        theta = 0.0;
        P = Eigen::MatrixXd::Identity(5, 5);
        Q = Eigen::MatrixXd::Identity(5, 5) * 0.01;
        R = Eigen::MatrixXd::Identity(5, 5) * 100;
    }
};

State state, old_state; 
nav_msgs::Path path_msg;
ros::Publisher path_pub, odom_pub;

std::deque<sensor_msgs::Imu::ConstPtr> imu_queue;
std::deque<nav_msgs::Odometry::ConstPtr> wheel_odom_queue;

double last_timestamp =0.0;
double last_wheel_time = 0.0;
bool initialized = false;
std::mutex state_mutex;
bool is_slipping = false;
sensor_msgs::Imu::ConstPtr last_imu_msg;
nav_msgs::Odometry::ConstPtr last_wheel_msg;
double last_imu_time = 0.0;

ros::Publisher imu_pub;
ros::Subscriber encoder_sub, imu_sub;
int encoder_resolution_ = 4096;
double left_wheel_diameter_ = 0.623803;  // 左轮直径（米）
double right_wheel_diameter_ = 0.623095; // 右轮直径（米）
double wheel_base_ = 1.52683;            // 轮距（米）

int prev_left_count_ = 0;
int prev_right_count_ = 0;
ros::Time prev_time_;
// 里程计变量
double x_ = 0.0;  // 机器人在 x 方向的累计位置
double y_ = 0.0;  // 机器人在 y 方向的累计位置
double theta_ = 0.0; // 机器人朝向（弧度）
std::mutex mutex_;

// IMU相关变量
double prev_linear_velocity_ = 0.0;  // 上一次IMU线速度
double prev_angular_velocity_ = 0.0; // 上一次IMU角速度
double slip_threshold_ = 3; // 打滑检测阈值
static int jjj = 0;
double normalizeAngle(double angle) {
    return atan2(sin(angle), cos(angle)); // 归一化到 [-π, π]
}
void encoderCallback(const irp_sen_msgs::encoder::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    // 获取时间戳
    ros::Time current_time = msg->header.stamp;

    // 当前编码器计数值
    int left_count = msg->left_count;
    int right_count = msg->right_count;

    // 如果是第一次回调，初始化计数和时间
    if (prev_time_.isZero()) {
        prev_left_count_ = left_count;
        prev_right_count_ = right_count;
        prev_time_ = current_time;
        return;
    }

    // 时间差
    double dt = (current_time - prev_time_).toSec();
    if (dt <= 0) {
        return; // 避免 dt 为负数或过小
    }

    // 编码器计数差分，处理溢出情况
    int delta_left_count = (left_count - prev_left_count_ + encoder_resolution_) % encoder_resolution_;
    int delta_right_count = (right_count - prev_right_count_ + encoder_resolution_) % encoder_resolution_;

    // 计算左右轮速度
    double v_left = (delta_left_count * M_PI * left_wheel_diameter_) / (encoder_resolution_ * dt);
    double v_right = (delta_right_count * M_PI * right_wheel_diameter_) / (encoder_resolution_ * dt);

    // 计算机器人线速度和角速度
    double linear_velocity = (v_left + v_right) / 2.0;
    double angular_velocity = (v_right - v_left) / wheel_base_;

    // 计算位姿更新（使用差分法）
    double delta_theta = angular_velocity * dt + 1e-5;
    theta_ += delta_theta;
    theta_= normalizeAngle(theta_);
    double delta_x = linear_velocity * cos(theta_) * dt;
    double delta_y = linear_velocity * sin(theta_) * dt;
    x_ += delta_x;
    y_ += delta_y;

    // 发布里程计消息
    nav_msgs::Odometry odom2_msg;
    odom2_msg.header.stamp =  current_time;
    odom2_msg.header.frame_id = "world";
    odom2_msg.child_frame_id = "base_link";

    odom2_msg.pose.pose.position.x = delta_x;
    odom2_msg.pose.pose.position.y = delta_y;
    odom2_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom2_msg.pose.pose.orientation.x = q.x();
    odom2_msg.pose.pose.orientation.y = q.y();
    odom2_msg.pose.pose.orientation.z = q.z();
    odom2_msg.pose.pose.orientation.w = q.w();

    odom2_msg.twist.twist.linear.x = linear_velocity;
    odom2_msg.twist.twist.linear.y = theta_;
    odom2_msg.twist.twist.linear.z = 0.0;

    odom2_msg.twist.twist.angular.x = 0.0;
    odom2_msg.twist.twist.angular.y = 0.0;
    odom2_msg.twist.twist.angular.z = angular_velocity;

    // 更新上一次的计数值和时间
    prev_left_count_ = left_count;
    prev_right_count_ = right_count;
    prev_time_ = current_time;
    prev_linear_velocity_ = linear_velocity;
    prev_angular_velocity_ = angular_velocity;

    boost::shared_ptr<const nav_msgs::Odometry> odom_ptr(new nav_msgs::Odometry(odom2_msg));
    wheel_odom_queue.push_back(odom_ptr);

}

// === 回调函数 ===
void imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(state_mutex);
    imu_queue.push_back(msg);
}

// === 发布路径 ===
void publishPath(const sensor_msgs::Imu::ConstPtr &imu_msg) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "world";

    pose_msg.pose.position.x = state.position(0);
    pose_msg.pose.position.y = state.position(1);
    pose_msg.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world";
    path_msg.poses.push_back(pose_msg);

    const size_t max_path_size = 1000000;
    if (path_msg.poses.size() > max_path_size) {
        path_msg.poses.erase(path_msg.poses.begin());
    }

    path_pub.publish(path_msg);
}

// === 发布里程计 ===
void publishOdometry() {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = state.position(0);
    odom_msg.pose.pose.position.y = state.position(1);
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = state.velocity(0);
    odom_msg.twist.twist.linear.y = state.velocity(1);
    odom_msg.twist.twist.linear.z = 0.0;

    odom_pub.publish(odom_msg);
}

// 计算旋转矩阵
Eigen::Matrix3d computeRotationMatrix(const Eigen::Vector3d &angular_velocity, double delta_t) {
    double angle = angular_velocity.norm() * delta_t;
    Eigen::Vector3d axis = angular_velocity.normalized();
    Eigen::AngleAxisd rotation_vector(angle, axis);
    return rotation_vector.toRotationMatrix();
}

// === 状态预测 ===
void predictState(const sensor_msgs::Imu::ConstPtr &imu_msg, double dt) {
    // 1. 提取 IMU 数据 (线加速度 + 角速度)
    Eigen::Vector3d linear_acc(imu_msg->linear_acceleration.x, 
                               imu_msg->linear_acceleration.y, 
                               imu_msg->linear_acceleration.z);
    // 使用四元数生成旋转矩阵
    Eigen::Quaterniond q(imu_msg->orientation.w, 
                         imu_msg->orientation.x, 
                         imu_msg->orientation.y, 
                         imu_msg->orientation.z);

    // Eigen::Vector3d linear_acc(0.0, 
    //                            0.0, 
    //                            0.0);
    // //使用四元数生成旋转矩阵
    // Eigen::Quaterniond q(1, 
    //                      0, 
    //                      0, 
    //                      0);

    Eigen::Matrix3d R = q.toRotationMatrix();

    Eigen::Vector3d gravity_imu(0, 0, GRAVITY);
    Eigen::Vector3d acc_world = R * linear_acc - gravity_imu;

    // 2. 计算角速度
    double imu_delta_angular = imu_msg->angular_velocity.z * dt; 
    //  double imu_delta_angular = 0.0 * dt; 
    double delta_angular = normalizeAngle(old_state.theta + imu_delta_angular);
    state.theta = delta_angular;

    // 3. 计算状态转移矩阵 F
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
    F(0, 2) = dt; 
    F(1, 3) = dt; 
    F(2, 4) = -state.velocity(1) * sin(state.theta) * dt; 
    F(3, 4) = state.velocity(0) * cos(state.theta) * dt; 

    // 6. 计算状态
    state.position = old_state.position + state.velocity * dt + 0.5 * acc_world.head<2>() * dt * dt;
    state.velocity = old_state.velocity + acc_world.head<2>() * dt;
    state.theta =  atan2(sin(state.theta), cos(state.theta));

    // 7. 计算协方差
    state.P = F * old_state.P * F.transpose() + state.Q;
}


Eigen::VectorXd stateToVector1() {
    Eigen::VectorXd x(5);
    x << state.position(0) - old_state.position(0), state.position(1) - old_state.position(1), state.velocity(0), state.velocity(1), state.theta;
    return x;
}
Eigen::VectorXd stateToVector2() {
    Eigen::VectorXd x(5);
    x << old_state.position(0), old_state.position(1), old_state.velocity(0), old_state.velocity(1), old_state.theta;
    return x;
}

void vectorToState(const Eigen::VectorXd &x) {
    state.position(0) = x(0);
    state.position(1) = x(1);
    state.velocity(0) = x(2);
    state.velocity(1) = x(3);
    state.theta = x(4);
}
// === 状态更新 ===
void updateObservation_eskf(const nav_msgs::Odometry::ConstPtr &wheel_msg) {
    double delta_t = wheel_msg->header.stamp.toSec() - last_wheel_time;

    double h_theta = wheel_msg->twist.twist.linear.y;

    double h_wheel_Vx = wheel_msg->twist.twist.linear.x;
    double delta_wheel_x = h_wheel_Vx * cos(h_theta) * delta_t;
    double delta_wheel_y = h_wheel_Vx * sin(h_theta) * delta_t;
    double h_wheel_x = old_state.position(0) + delta_wheel_x;
    double h_wheel_y = old_state.position(1) + delta_wheel_x;
    Eigen::VectorXd z(5);
    z<< h_wheel_x,
        h_wheel_y, 
        h_wheel_Vx * cos(h_theta) , 
        h_wheel_Vx * sin(h_theta) , 
        h_theta;

    Eigen::Matrix<double, 5, 5> H;
    H.setZero();
    H(0, 0) = 1;
    H(1, 1) = 1;
    H(2, 2) = 1; // 对 vx 的影响
    H(3, 3) = 1; // 对 vy 的影响 
    H(3, 4) = 1;                // 对 theta 的影响
    Eigen::VectorXd y = z - H * stateToVector2();
    // ROS_INFO_STREAM("y: " << y.transpose());
    Eigen::MatrixXd S = H * state.P * H.transpose() + state.R; //5*5 * 5*5 * 5*5 + 5*5 = 5*5
    Eigen::MatrixXd K = state.P * H.transpose() * S.inverse(); //5*5 * 5*5 * 5*5 = 5*5

    Eigen::VectorXd x_new = stateToVector2() + K * y; // 更新后的状态向量 5*1 +5*5*4*1 =5*1
    vectorToState(x_new); // 转换回状态结构

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
    state.P = (I - K * H) * state.P * (I - K * H).transpose() + K * state.R * K.transpose();
    last_wheel_time = wheel_msg->header.stamp.toSec();
}

void syncData() {
    if(imu_queue.size() < 50 && wheel_odom_queue.size()< 50 ){
        return;
    }
    else if(!imu_queue.empty() && !wheel_odom_queue.empty()) {
        // std::cout<<"imu_queue.size() ="<<imu_queue.size()<<std::endl;
        // std::cout<<"wheel_odom_queue.size() ="<<wheel_odom_queue.size()<<std::endl;
        sensor_msgs::Imu::ConstPtr imu_msg = imu_queue.front();
        nav_msgs::Odometry::ConstPtr wheel_msg = wheel_odom_queue.front();
        
        if (!initialized && wheel_msg->twist.twist.linear.x != 0) {
            // 假设初始状态时机器人静止且位于原点
            state.position(0) = 0.0;
            state.position(1) = 0.0;
            state.velocity(0) = 0.0;
            state.velocity(1) = 0.0;
            state.theta = 0.0;
            last_imu_msg = imu_queue.front();
            last_imu_time = last_imu_msg->header.stamp.toSec();
            last_wheel_msg = wheel_odom_queue.front();
            last_wheel_time = last_wheel_msg->header.stamp.toSec();
            // // 基于IMU加速度估计初始速度

            // 初始协方差矩阵设定较大值，以便逐步调整
            state.P = Eigen::MatrixXd::Identity(5, 5) * 10;
            state.R.block<2, 2>(0, 0) = Eigen::MatrixXd::Identity(2, 2) * 0.1; // 高协方差，允许不确定性
            state.R.block<2, 2>(2, 2) = Eigen::MatrixXd::Identity(2, 2) * 0.1;
            state.R.block<1, 1>(4, 4) = Eigen::MatrixXd::Identity(1, 1) * 0.1;

            old_state = state; // 更新初始状态

            ROS_INFO("Initialized!");
            initialized = true;

            imu_queue.pop_front();
            wheel_odom_queue.pop_front();
        }
        else if(initialized && !imu_queue.empty() && !wheel_odom_queue.empty()){
            updateObservation_eskf(wheel_msg);
            wheel_odom_queue.pop_front(); // 丢弃轮速计数据    
            // 检查时间戳差值并同步
            while (imu_queue.front()->header.stamp.toSec() < last_wheel_time) {
                double delta_t = imu_queue.front()->header.stamp.toSec() - last_imu_time;

                // 状态预测与更新
                predictState(imu_queue.front(), delta_t);

                // 更新时间戳
                last_imu_time = imu_queue.front()->header.stamp.toSec();

                last_imu_msg = imu_queue.front();
                // 移除已处理的数据
                ROS_INFO_STREAM("imu_queue.size(): " << imu_queue.size());
                imu_queue.pop_front();

            }   
           
            // 更新历史状态
            old_state = state;
            // 发布路径与里程计
            publishPath(imu_queue.front());
            publishOdometry();
        }
        else {
            imu_queue.clear();
            wheel_odom_queue.clear();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "imu_wheel_fusion_node2");
    ros::NodeHandle nh;

    // 初始化状态
    state.position = Eigen::Vector2d::Zero();
    state.velocity = Eigen::Vector2d::Zero();
    state.theta = 0.0;
    state.P = Eigen::MatrixXd::Identity(5, 5) * 1.0;

    // 订阅 IMU 和轮速计话题
    ros::Subscriber imu_sub = nh.subscribe("/imu/data_raw", 100, imuCallback);
    // ros::Subscriber odom_sub = nh.subscribe("/ridgeback_velocity_controller/odom", 100, wheelOdomCallback);
    encoder_sub = nh.subscribe("/encoder_count", 10, &encoderCallback);
    // imu_sub = nh.subscribe("/xsens_imu_data", 10, imuCallback);
    // 发布 Path 和 Odometry
    path_pub = nh.advertise<nav_msgs::Path>("/path2", 100);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry", 100);

    ros::Rate rate(5000000);
    // 定时器用于调用同步数据处理函数
    ros::Timer timer = nh.createTimer(ros::Duration(0.00001), [](const ros::TimerEvent&) { syncData(); });

    // 启动回调处理
    ros::spin();

    rate.sleep();

    return 0;
}
