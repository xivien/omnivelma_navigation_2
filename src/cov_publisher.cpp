#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cinttypes>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class CovPub : public rclcpp::Node
{
public:
    CovPub() : Node("cov_pub_node")
    {

        pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/noisy/cov", 10);
        pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/cov", 10);

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/noisy", 10, std::bind(&CovPub::odom_callback, this, std::placeholders::_1));

        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/filtered", 10, std::bind(&CovPub::imu_callback, this, std::placeholders::_1));

    }

private:
    void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
    {
        msg->pose.covariance[0] = 0.001;
        msg->pose.covariance[7] = 0.001;
        msg->pose.covariance[35] = 0.01;

        msg->twist.covariance[0] = 0.0002;
        msg->twist.covariance[7] = 0.0002;
        msg->twist.covariance[35] = 0.001;

        pub_odom_->publish(*msg);
    }

    void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg)
    {
        msg->orientation_covariance[8] = 0.001;
        msg->angular_velocity_covariance[8] = 1e-6;
        msg->linear_acceleration_covariance[0] = 1e-5;
        msg->linear_acceleration_covariance[4] = 1e-5;

        pub_imu_->publish(*msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CovPub>());
    rclcpp::shutdown();
    return 0;
}