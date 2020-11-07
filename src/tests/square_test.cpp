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

class SquareTest : public rclcpp::Node
{
public:
    SquareTest() : Node("square_test_node"), loop_rate_(20.0f), freq_(20.0f), mode_(1), dest_x_(2), dest_y_(2)
    {
        rclcpp::sleep_for(2s);

        pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&SquareTest::odom_callback, this, std::placeholders::_1));
        amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10, std::bind(&SquareTest::pose_callback, this, std::placeholders::_1));

        rclcpp::sleep_for(2s);

        destination_x_ = {dest_x_, dest_x_, 0.0f, 0.0f};
        destination_y_ = {0.0f, -dest_y_, -dest_y_, 0.0f};

        switch (mode_)
        {
        case 1:
            do_mode_1();
            break;
        case 2:
            do_mode_2();
            break;

        default:
            break;
        }
    }

private:
    void do_mode_1()
    {
        geometry_msgs::msg::Twist vel_msg;

        for (size_t i = 0; i < 4; i++)
        {
            float last_direct = 10000.0f;
            float last_dist = 10000.0f;

            float dx, dy;
            dx = destination_x_[i] - pos_x_;
            dy = destination_y_[i] - pos_y_;

            float direct = std::atan2(dy, dx) - yaw_;
            last_direct = direct;

            while (true)
            {
                vel_msg.linear.x = 0.0f;
                vel_msg.linear.y = 0.0f;
                vel_msg.angular.z = -vel_;
                pub_vel_->publish(vel_msg);
                loop_rate_.sleep();
                rclcpp::spin_some(this->get_node_base_interface());

                last_direct = direct;

                direct = std::atan2(dy, dx) - yaw_;

                //cast to (-pi:pi)
                if (direct < -M_PI)
                {
                    direct += 2.0f * M_PI;
                }
                else if (direct > M_PI)
                {
                    direct -= 2.0f * M_PI;
                }

                if (std::abs(direct) < 0.1f && std::abs(direct) > std::abs(last_direct))
                {
                    break;
                }
            }
            vel_msg.angular.z = 0.0f;
            vel_msg.linear.x = vel_;
            for (int i = 0; i < 5; i++)
            {
                pub_vel_->publish(vel_msg);
                loop_rate_.sleep();
                rclcpp::spin_some(this->get_node_base_interface());
            }
            dx = destination_x_[i] - pos_x_;
            dy = destination_y_[i] - pos_y_;
            float dist = std::sqrt(dx * dx + dy * dy);

            while (std::abs(dist) <= std::abs(last_dist))
            {
                vel_msg.linear.x = vel_;
                pub_vel_->publish(vel_msg);
                loop_rate_.sleep();
                rclcpp::spin_some(this->get_node_base_interface());

                last_dist = dist;
                dx = destination_x_[i] - pos_x_;
                dy = destination_y_[i] - pos_y_;
                dist = std::sqrt(dx * dx + dy * dy);
            }
            vel_msg.linear.x = 0.0f;
            vel_msg.linear.x = vel_;
            pub_vel_->publish(vel_msg);
            loop_rate_.sleep();
        }
        // Turn to starting pose
        float last_direct = 100000.0f;
        float direct = -yaw_;
        while (true)
        {
            vel_msg.linear.x = 0.0f;
            vel_msg.linear.y = 0.0f;
            vel_msg.angular.z = -vel_;
            pub_vel_->publish(vel_msg);
            loop_rate_.sleep();
            rclcpp::spin_some(this->get_node_base_interface());

            last_direct = direct;

            direct = -yaw_;

            //cast to (-pi:pi)
            if (direct < -M_PI)
            {
                direct += 2.0f * M_PI;
            }
            else if (direct > M_PI)
            {
                direct -= 2.0f * M_PI;
            }

            if (std::abs(direct) < 0.1f && std::abs(direct) > std::abs(last_direct))
            {
                break;
            }
        }
        vel_msg.angular.z = 0.0f;
        pub_vel_->publish(vel_msg);
        loop_rate_.sleep();
        rclcpp::spin_some(this->get_node_base_interface());
    }
    void do_mode_2()
    {
        geometry_msgs::msg::Twist vel_msg;

        for (size_t i = 0; i < 4; i++)
        {
            float last_dx = 10000.0f;
            float last_dy = 10000.0f;

            float dx, dy;
            dx = destination_x_[i] - pos_x_;
            dy = destination_y_[i] - pos_y_;

            while ((std::abs(dx) <= std::abs(last_dx) && std::abs(dy) <= std::abs(last_dy)) ||
                   std::abs(dx) > tol_ || std::abs(dy) >= tol_)
            {
                vel_msg.linear.x = vel_ * dx / std::sqrt(dx * dx + dy * dy);
                vel_msg.linear.y = vel_ * dy / std::sqrt(dx * dx + dy * dy);

                pub_vel_->publish(vel_msg);
                loop_rate_.sleep();
                rclcpp::spin_some(this->get_node_base_interface());

                last_dx = dx;
                last_dy = dy;
                dx = destination_x_[i] - pos_x_;
                dy = destination_y_[i] - pos_y_;
            }
            vel_msg.linear.x = 0.0f;
            vel_msg.linear.y = 0.0f;
            pub_vel_->publish(vel_msg);
            loop_rate_.sleep();
            rclcpp::spin_some(this->get_node_base_interface());
        }
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
        pos_x_ = msg->pose.pose.position.x;
        pos_y_ = msg->pose.pose.position.y;
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
        pos_x_ = msg->pose.pose.position.x;
        pos_y_ = msg->pose.pose.position.y;
        **/
    }

    rclcpp::Rate loop_rate_;
    float freq_;
    float vel_ = 0.3f;
    double yaw_, pos_x_, pos_y_;
    int mode_;
    float dest_x_, dest_y_;
    float tol_ = 0.1f;

    std::array<float, 4> destination_x_;
    std::array<float, 4> destination_y_;

    nav_msgs::msg::Odometry::SharedPtr odom_;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareTest>());
    rclcpp::shutdown();
    return 0;
}