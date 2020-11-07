#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cinttypes>
#include <future>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class TurnTest : public rclcpp::Node
{
public:
    TurnTest() : Node("turn_test_node"), loop_rate_(20.0f), freq_(20.0f), destination_(4.0f * M_PI)
    {
        rclcpp::sleep_for(2s);

        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&TurnTest::odom_callback, this, std::placeholders::_1));
        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&TurnTest::pose_callback, this, std::placeholders::_1));

        rclcpp::sleep_for(2s);
        do_trajectory();
    }

private:
    void do_trajectory()
    {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.angular.z = vel_;
        float last_theta = yaw_;
        float spinned = 0.0f;

        while ((yaw_ + spinned) < destination_)
        {
            pub_vel_->publish(vel_msg);
            loop_rate_.sleep();
            rclcpp::spin_some(this->get_node_base_interface());

            if (yaw_ < last_theta)
            {
                spinned += 2.0f * M_PI;
            }
            last_theta = yaw_;
        }
        vel_msg.angular.z = 0.0f;
        pub_vel_->publish(vel_msg);
        loop_rate_.sleep();
        rclcpp::spin_some(this->get_node_base_interface());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //when testing with Odometry
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // when testing with AMCL position
        /**
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_);
        **/
    }

    rclcpp::Rate loop_rate_;
    float freq_;
    float vel_ = 0.3f;
    double yaw_;
    float destination_;

    nav_msgs::msg::Odometry::SharedPtr odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurnTest>());
    rclcpp::shutdown();
    return 0;
}