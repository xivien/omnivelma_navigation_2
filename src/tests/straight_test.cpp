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
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class StraightTest : public rclcpp::Node
{
public:
    StraightTest() : Node("straight_test_node"), loop_rate_(20.0f), freq_(20.0f)
    {
        rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true)); // Set to false for real robot
        set_parameter(simTime);

        rclcpp::sleep_for(2s);

        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&StraightTest::odom_callback, this, std::placeholders::_1));
        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&StraightTest::pose_callback, this, std::placeholders::_1));

        rclcpp::sleep_for(2s);

        do_trajectory();
    }

private:
    void do_trajectory()
    {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = vel_;
        float last_dist = 10000.0f;
        float destination = 2.0f;

        rclcpp::spin_some(this->get_node_base_interface());

        float dist = destination - pos_x_;
        while (std::abs(dist) <= std::abs(last_dist))
        {
            pub_vel_->publish(vel_msg);
            last_dist = dist;

            loop_rate_.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
            dist = destination - pos_x_;
        }
        vel_msg.linear.x = 0.0f;
        pub_vel_->publish(vel_msg);

        loop_rate_.sleep();
        rclcpp::spin_some(this->get_node_base_interface());

        destination = 0.0f;
        last_dist = 10000.0f;
        dist = pos_x_;

        vel_msg.linear.x = -vel_;
        while (std::abs(dist) <= std::abs(last_dist))
        {
            pub_vel_->publish(vel_msg);
            last_dist = dist;

            loop_rate_.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
            dist = destination - pos_x_;
        }
        vel_msg.linear.x = 0.0f;
        pub_vel_->publish(vel_msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //when testing with Odometry
        pos_x_ = msg->pose.pose.position.x;
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // when testing with AMCL position
        // pos_x_ = msg->pose.pose.position.x;
    }

    rclcpp::Rate loop_rate_;
    float freq_;
    float vel_ = 0.3f;
    float pos_x_;

    nav_msgs::msg::Odometry::SharedPtr odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StraightTest>());
    rclcpp::shutdown();
    return 0;
}