#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cinttypes>
#include <future>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

using namespace std::chrono_literals;

class WaypointFollower : public rclcpp::Node
{
public:
    using FollowWaypointsAction = nav2_msgs::action::FollowWaypoints;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypointsAction>;
    WaypointFollower() : Node("waypoint_follower")
    {
        this->declare_parameter("waypoints");
        rclcpp::Parameter double_array_param = this->get_parameter("waypoints");
        waypoints = double_array_param.as_double_array();

        if (waypoints.size() % 3 != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Waypoints have to be 3 element vectors");
        }

        client_ptr_ = rclcpp_action::create_client<FollowWaypointsAction>(this, "FollowWaypoints");
        rclcpp::spin_some(this->get_node_base_interface());
        send_waypoints();
    }

private:
    void send_waypoints()
    {
        if (!this->client_ptr_)
        {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }
        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            return;
        }

        auto send_goal_options = rclcpp_action::Client<FollowWaypointsAction>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&WaypointFollower::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&WaypointFollower::result_callback, this, std::placeholders::_1);
        auto new_goal = FollowWaypointsAction::Goal();

        geometry_msgs::msg::PoseStamped goal_msg;
        goal_msg.header.frame_id = "map";
        for (size_t i = 0; i < waypoints.size(); i = i + 3)
        {
            RCLCPP_INFO(this->get_logger(), "Waypoint added x:%f y:%f th:%f", waypoints[i], waypoints[i + 1], waypoints[i + 2]);
            goal_msg.pose.position.x = waypoints[i];
            goal_msg.pose.position.y = waypoints[i + 1];
            goal_msg.pose.orientation.z = waypoints[i + 2];
            new_goal.poses.emplace_back(goal_msg);
        }

        auto goal_handle_future = this->client_ptr_->async_send_goal(new_goal, send_goal_options);
    }

    void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const FollowWaypointsAction::Feedback> feedback)
    {
        feedback_it_++;
        if (feedback_it_ % 10 == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Current waypoint: %d", feedback->current_waypoint);
            feedback_it_ = 0;
        }
    }

    void result_callback(const GoalHandle::WrappedResult &result)
    {
        // this->goal_done_ = true;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Result received, Missed waypoints:");
        if (result.result->missed_waypoints.size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "None");
        }
        else
        {
            for (auto number : result.result->missed_waypoints)
            {
                RCLCPP_INFO(this->get_logger(), "%d", number);
            }
        }
    }

    rclcpp_action::Client<FollowWaypointsAction>::SharedPtr client_ptr_;
    int feedback_it_ = 0;
    std::vector<double> waypoints;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointFollower>());
    rclcpp::shutdown();
    return 0;
}