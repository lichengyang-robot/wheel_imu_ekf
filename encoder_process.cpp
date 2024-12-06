#include <ros/ros.h>
#include <irp_sen_msgs/encoder.h>
#include <geometry_msgs/TwistStamped.h>
#include <mutex>
#include <ros/ros.h>
#include <irp_sen_msgs/imu.h>
#include <sensor_msgs/Imu.h>

ros::Publisher imu_pub;
    ros::Subscriber encoder_sub_;
    ros::Publisher velocity_pub_;
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


        double encoder_resolution_ = 4096.0;
        double left_wheel_diameter_ = 0.623803;  // 左轮直径（米）
        double right_wheel_diameter_ = 0.623095; // 右轮直径（米）
        double wheel_base_ = 1.52683;            // 轮距（米）

    int prev_left_count_ = 0;
    int prev_right_count_ = 0;
    ros::Time prev_time_;

    std::mutex mutex_;
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

        // 编码器计数差分
        int delta_left_count = left_count - prev_left_count_;
        int delta_right_count = right_count - prev_right_count_;

        // 计算左右轮速度
        double v_left = (delta_left_count * M_PI * left_wheel_diameter_) / (encoder_resolution_ * dt);
        double v_right = (delta_right_count * M_PI * right_wheel_diameter_) / (encoder_resolution_ * dt);

        // 计算机器人线速度和角速度
        double linear_velocity = (v_left + v_right) / 2.0;
        double angular_velocity = (v_right - v_left) / wheel_base_;

        // 发布 TwistStamped 消息
        geometry_msgs::TwistStamped twist_msg;
        twist_msg.header.stamp = current_time;
        twist_msg.twist.linear.x = linear_velocity;
        twist_msg.twist.angular.z = angular_velocity;

        velocity_pub_.publish(twist_msg);

        // 更新上一次的计数值和时间
        prev_left_count_ = left_count;
        prev_right_count_ = right_count;
        prev_time_ = current_time;
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "encoder_processor");
    ros::NodeHandle nh_;
    // 订阅 irp_sen_msgs/imu 消息
        // 初始化参数

        // 订阅 encoder_count 话题
        encoder_sub_ = nh_.subscribe("/encoder_count", 10, &encoderCallback);

        // 发布 TwistStamped 消息
        velocity_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/wheel_odom", 10);
    ros::Subscriber imu_sub = nh_.subscribe("/xsens_imu_data", 10, imuCallback);

    // 创建 sensor_msgs/Imu 的发布者
    imu_pub = nh_.advertise<sensor_msgs::Imu>("/imu/data", 10);
    ros::spin();
    return 0;
}
