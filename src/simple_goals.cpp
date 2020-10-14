#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cinttypes>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class SimpleGoals : public rclcpp::Node
{
public:
    SimpleGoals() : Node("simple_navigation_goals")
    {
        pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
        bool isGoal = true;
        while (isGoal)
        {
            rclcpp::spin_some(this->get_node_base_interface());
            isGoal = send_goal();
        }
        RCLCPP_INFO(this->get_logger(), "Finished running");
    }

private:
    bool send_goal()
    {
        float goal_x_, goal_y_, goal_theta_;
        RCLCPP_INFO(this->get_logger(), "To exit use invalid input");
        RCLCPP_INFO(this->get_logger(), "Send navigation goal X[m] Y[m] Theta[rad]");
        std::cin >> goal_x_ >> goal_y_ >> goal_theta_;
        if (!std::cin)
        {
            RCLCPP_INFO(this->get_logger(), "Exiting mode");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Navigating to a goal x=%.2f y=%.2f theta=%.2f", goal_x_, goal_y_, goal_theta_);

        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        goal_msg.pose.position.x = goal_x_;
        goal_msg.pose.position.y = goal_y_;
        goal_msg.pose.orientation.z = goal_theta_;
        pub_goal_->publish(goal_msg);
        return true;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleGoals>());
    rclcpp::shutdown();
    return 0;
}