#include <ros/ros.h>
#include <irp_sen_msgs/encoder.h>
#include <geometry_msgs/TwistStamped.h>
#include <mutex>
#include <ros/ros.h>
#include <irp_sen_msgs/imu.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <irp_sen_msgs/encoder.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <mutex>
#include <nav_msgs/Path.h>
ros::Publisher imu_pub;
ros::Subscriber encoder_sub_;
ros::Publisher odom_pub;
int encoder_resolution_ = 4096;
double left_wheel_diameter_ = 0.623803;  // 左轮直径（米）
double right_wheel_diameter_ = 0.623095; // 右轮直径（米）
double wheel_base_ = 1.52683;            // 轮距（米）

int prev_left_count_ = 0;
int prev_right_count_ = 0;
ros::Time prev_time_;
// ros::Publisher velocity_pub_; // 发布速度消息
ros::Publisher path_pub; // 发布里程计消息

// 里程计变量
double x_ = 0.0;  // 机器人在 x 方向的累计位置
double y_ = 0.0;  // 机器人在 y 方向的累计位置
double theta_ = 0.0; // 机器人朝向（弧度）
nav_msgs::Path path_msg;
std::mutex mutex_;
// 回调函数：将 irp_sen_msgs/imu 转换为 sensor_msgs/Imu
void imuCallback(const irp_sen_msgs::imu::ConstPtr& msg)
{
    // 初始化 sensor_msgs/Imu 消息
    sensor_msgs::Imu converted_msg;

    // 1. 复制 header
    converted_msg.header = msg->header;

    // 2. 复制 orientation（使用 quaternion_data）
    converted_msg.orientation = msg->quaternion_data;

    // 3. 复制 angular velocity（使用 gyro_data）
    converted_msg.angular_velocity.x = msg->gyro_data.x;
    converted_msg.angular_velocity.y = msg->gyro_data.y;
    converted_msg.angular_velocity.z = msg->gyro_data.z;

    // 4. 复制 linear acceleration（使用 acceleration_data）
    converted_msg.linear_acceleration.x = msg->acceleration_data.x;
    converted_msg.linear_acceleration.y = msg->acceleration_data.y;
    converted_msg.linear_acceleration.z = msg->acceleration_data.z;

    // 5. (可选) 填充 covariance（如果需要）
    // 如果原始消息未提供协方差，可以根据实际需要手动设置
    for (int i = 0; i < 9; ++i) {
        converted_msg.orientation_covariance[i] = 0.0; // 暂时置为零
        converted_msg.angular_velocity_covariance[i] = 0.0; // 暂时置为零
        converted_msg.linear_acceleration_covariance[i] = 0.0; // 暂时置为零
    }

    // 发布转换后的消息
    imu_pub.publish(converted_msg);
}


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

    nav_msgs::Odometry odom2_msg;
    odom2_msg.header.stamp =  current_time;
    odom2_msg.header.frame_id = "world";
    odom2_msg.child_frame_id = "base_link";

    odom2_msg.pose.pose.position.x = x_;
    odom2_msg.pose.pose.position.y = y_;
    odom2_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom2_msg.pose.pose.orientation.x = q.x();
    odom2_msg.pose.pose.orientation.y = q.y();
    odom2_msg.pose.pose.orientation.z = q.z();
    odom2_msg.pose.pose.orientation.w = q.w();

    odom2_msg.twist.twist.linear.x = linear_velocity * cos(theta_);
    odom2_msg.twist.twist.linear.y = linear_velocity * sin(theta_);
    odom2_msg.twist.twist.linear.z = 0.0;

    odom2_msg.twist.twist.angular.x = 0.0;
    odom2_msg.twist.twist.angular.y = 0.0;
    odom2_msg.twist.twist.angular.z = angular_velocity;

    odom_pub.publish(odom2_msg);

    // 发布里程计消息
    geometry_msgs::PoseStamped odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = "world"; // 固定坐标系
    // odom_msg.child_frame_id = "base_link"; // 机器人基座坐标系

    // 设置位置
    odom_msg.pose.position.x = x_;
    odom_msg.pose.position.y = y_;
    odom_msg.pose.position.z = 1.0;
    tf::Quaternion odom_quat = tf::createQuaternionFromYaw(theta_);
    odom_msg.pose.orientation.x = odom_quat.x();
    odom_msg.pose.orientation.y = odom_quat.y();
    odom_msg.pose.orientation.z = odom_quat.z();
    odom_msg.pose.orientation.w = odom_quat.w();
    
    path_msg.header.stamp =  current_time;
    path_msg.header.frame_id = "world";
    path_msg.poses.push_back(odom_msg);

    const size_t max_path_size = 1000000;
    if (path_msg.poses.size() > max_path_size) {
        path_msg.poses.erase(path_msg.poses.begin());
    }

    path_pub.publish(path_msg);

    // 更新上一次的计数值和时间
    prev_left_count_ = left_count;
    prev_right_count_ = right_count;
    prev_time_ = current_time;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "encoder_processor");
    ros::NodeHandle nh_;

    // 订阅 encoder_count 话题
    encoder_sub_ = nh_.subscribe("/encoder_count", 10, &encoderCallback);

    // 发布 TwistStamped 消息
        path_pub = nh_.advertise<nav_msgs::Path>("/robot_path", 100);
        odom_pub = nh_.advertise<nav_msgs::Odometry>("/robot_odom", 100);
    // ros::Subscriber imu_sub = nh_.subscribe("/xsens_imu_data", 10, imuCallback);

    // // 创建 sensor_msgs/Imu 的发布者
    // imu_pub = nh_.advertise<sensor_msgs::Imu>("/imu/data", 10);
    ros::spin();
    return 0;
}