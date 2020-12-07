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
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class ErrorPub : public rclcpp::Node
{
public:
    ErrorPub() : Node("error_pub_node")
    {
        rclcpp::sleep_for(1s);

        pub_err_odom_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/error/odom", 10);
        pub_err_odom_filtered_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/error/odom_filtered", 10);
        pub_err_amcl_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/error/amcl", 10);
        pub_err_amcl_filtered_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/error/amcl_filtered", 10);

        filtered_odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&ErrorPub::odom_filtered_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/noisy", 10, std::bind(&ErrorPub::odom_callback, this, std::placeholders::_1));

        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&ErrorPub::amcl_callback, this, std::placeholders::_1));
        filtered_amcl_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/amcl/filtered", 10, std::bind(&ErrorPub::amcl_filtered_callback, this, std::placeholders::_1));

        perfect_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/omnivelma/pose", 10, std::bind(&ErrorPub::perfect_pose_callback, this, std::placeholders::_1));

        rclcpp::sleep_for(1s);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped error;
        error.header = msg->header;
        error.pose.position.x = msg->pose.pose.position.x - perfect_x_;
        error.pose.position.y = msg->pose.pose.position.y - perfect_y_;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        error.pose.orientation.z = yaw - perfect_yaw_;

        pub_err_odom_->publish(error);
    }

    void odom_filtered_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped error;
        error.header = msg->header;
        error.pose.position.x = msg->pose.pose.position.x - perfect_x_;
        error.pose.position.y = msg->pose.pose.position.y - perfect_y_;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        error.pose.orientation.z = yaw - perfect_yaw_;

        pub_err_odom_filtered_->publish(error);
    }

    void amcl_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped error;
        error.header = msg->header;
        error.pose.position.x = msg->pose.pose.position.x - perfect_x_;
        error.pose.position.y = msg->pose.pose.position.y - perfect_y_;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        error.pose.orientation.z = yaw - perfect_yaw_;
        // zero if error > pi

        pub_err_amcl_->publish(error);
    }

    void amcl_filtered_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped error;
        error.header = msg->header;
        error.pose.position.x = msg->pose.pose.position.x - perfect_x_;
        error.pose.position.y = msg->pose.pose.position.y - perfect_y_;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        error.pose.orientation.z = yaw - perfect_yaw_;

        pub_err_amcl_filtered_->publish(error);
    }

    void perfect_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        perfect_x_ = msg->pose.position.x;
        perfect_y_ = msg->pose.position.y;
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, perfect_yaw_);
    }

    double perfect_x_, perfect_y_;
    double perfect_yaw_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr filtered_amcl_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr perfect_pose_subscriber_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_err_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_err_odom_filtered_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_err_amcl_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_err_amcl_filtered_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorPub>());
    rclcpp::shutdown();
    return 0;
}