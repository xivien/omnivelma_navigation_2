#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cinttypes>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_waypoint_follower/waypoint_follower.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

using namespace std::chrono_literals;

class SimpleNavigationGoals : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  SimpleNavigationGoals() : Node("simple_navigation_goals"), loop_rate_(10.0f), freq_(10.0f), cov_tol_(0.1)
  {
    rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true)); // Set to false for real robot
    set_parameter(simTime);

    reinit_client_ = this->create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
    clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>("/global_costmap/clear_around_global_costmap");
    pub_init_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
    amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 1, std::bind(&SimpleNavigationGoals::pose_callback, this, std::placeholders::_1));

    client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(this, "FollowWaypoints");

    RCLCPP_INFO(this->get_logger(), "MAKE SURE THAT ROBOT HAS AN EMPTY 1mX1m AREA IN FRONT OF IT");
    RCLCPP_INFO(this->get_logger(), "Select mode: \n"
                                    "0 - reset localization and do homing sequence \n"
                                    "1 - set manual localization and do homing sequence \n"
                                    "2 - no homing sequence start with localization 0.0 \n");

    std::cin >> mode_;

    if (mode_ == 0)
    {
      reset_amcl();
    }
    else if (mode_ == 1)
    {
      init_pose();
    }
    if (mode_ < 2)
    {
      do_homing_sequence();
    }

    // check quality of localization
    rclcpp::spin_some(this->get_node_base_interface());
    clear_costmap();
    if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
    {
      RCLCPP_INFO(this->get_logger(), "High covariance \n The robot may crash \n"
                                      "Aborting navigation");
      RCLCPP_INFO(this->get_logger(), "Use teleop_twist_keyboard to drive manually and improve localization");
    }
    else
    {
      // bool is_goal = true;
      send_waypoints();
      // while (is_goal)
      // {
      //   is_goal = send_goal();
      // }
    }
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    cov_x_ = msg->pose.covariance[0];
    cov_y_ = msg->pose.covariance[7];
  }

  void clear_costmap()
  {
    auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();

    while (!clear_costmap_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto result = clear_costmap_client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      // RCLCPP_INFO(this->get_logger(), "Succeeded calling service clear cost map");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service clear cost map");
    }
  }

  void reset_amcl()
  {
    RCLCPP_INFO(this->get_logger(), "Resetting AMCL POSE");
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();

    while (!reinit_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto result = reinit_client_->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Succeeded calling service clear cost map");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service clear cost map");
    }
  }

  void init_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.orientation.w = 1;
    RCLCPP_INFO(this->get_logger(), "Enter X[m] Y[m] Theta[rad]");
    std::cin >> init_pose.pose.pose.position.x >> init_pose.pose.pose.position.y >> init_pose.pose.pose.orientation.z;

    // init covariances
    init_pose.pose.covariance[0] = 0.25;
    init_pose.pose.covariance[7] = 0.25;
    init_pose.pose.covariance[14] = 0.0625;
    RCLCPP_INFO(this->get_logger(), "Setting pose to x=%.2f y=%.2f theta=%.2f", init_pose.pose.pose.position.x,
                init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z);

    pub_init_pose_->publish(init_pose);
    rclcpp::spin_some(this->get_node_base_interface());
  }

  void do_homing_sequence()
  {
    // simple controls without checking for odometry
    float vel = 0.2f;
    float omega = 0.25f;
    float dist = 1.0f;

    int mov_steps = freq_ * dist / vel;
    int rot_steps = freq_ * 2 * M_PI / omega;

    geometry_msgs::msg::Twist new_msg;

    std::array<int, 4> dir_x = {1, 0, -1, 0};
    std::array<int, 4> dir_y = {0, -1, 0, 1};

    RCLCPP_INFO(this->get_logger(), "Homing sequence initiated ... rotating");
    for (int i = 0; i < rot_steps; i++)
    {
      new_msg.angular.z = omega;
      pub_vel_->publish(new_msg);
      loop_rate_.sleep();
      rclcpp::spin_some(this->get_node_base_interface());
    }

    new_msg.angular.z = 0.0f;
    pub_vel_->publish(new_msg);
    loop_rate_.sleep();
    rclcpp::spin_some(this->get_node_base_interface());
    clear_costmap();

    if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
    {
      RCLCPP_INFO(this->get_logger(), "High covariance, doing a square 1x1m");
      for (int i = 0; i < 4; i++)
      {
        for (int j = 0; j < mov_steps; j++)
        {
          new_msg.linear.x = vel * dir_x[i];
          new_msg.linear.y = vel * dir_y[i];
          pub_vel_->publish(new_msg);
          rclcpp::spin_some(this->get_node_base_interface());
          loop_rate_.sleep();
        }
      }

      new_msg.linear.x = 0.0f;
      new_msg.linear.y = 0.0f;
      pub_vel_->publish(new_msg);
      rclcpp::spin_some(this->get_node_base_interface());
      clear_costmap();
      loop_rate_.sleep();
    }

    if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
    {
      RCLCPP_INFO(this->get_logger(), "rotating again");
      for (int i = 0; i < rot_steps; i++)
      {
        new_msg.angular.z = omega;
        pub_vel_->publish(new_msg);
        loop_rate_.sleep();
        rclcpp::spin_some(this->get_node_base_interface());
      }
      new_msg.angular.z = 0.0f;
      pub_vel_->publish(new_msg);
      loop_rate_.sleep();
      rclcpp::spin_some(this->get_node_base_interface());
      clear_costmap();
    }
  }

  bool send_goal()
  {
    RCLCPP_INFO(this->get_logger(), "To exit use invalid input or press ctrl-c");
    RCLCPP_INFO(this->get_logger(), "Send navigation goal X[m] Y[m] Theta[rad]");
    // ros::Duration(1.0).sleep();
    std::cin >> goal_x_ >> goal_y_ >> goal_theta_;
    if (!std::cin)
    {
      RCLCPP_INFO(this->get_logger(), "Exiting \n Press ctrl+c to quit the program");
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

  void send_waypoints()
  {
    if (!this->client_ptr_)
    {
      RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
    }
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      // this->goal_done_ = true;
      return;
    }

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&SimpleNavigationGoals::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&SimpleNavigationGoals::result_callback, this, std::placeholders::_1);
    geometry_msgs::msg::PoseStamped goal_msg;

    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = 6.5;
    goal_msg.pose.position.y = -1;
    goal_msg.pose.orientation.z = 0;

    geometry_msgs::msg::PoseStamped goal_msg_2;
    goal_msg_2.header.frame_id = "map";
    goal_msg_2.pose.position.x = 0;
    goal_msg_2.pose.position.y = -1;
    goal_msg_2.pose.orientation.z = 0;

    auto new_goal = nav2_msgs::action::FollowWaypoints::Goal();

    for (int i = 0; i < 2; i++)
    {
      new_goal.poses.emplace_back(i % 2 ? goal_msg : goal_msg_2);
    }

    auto goal_handle_future = this->client_ptr_->async_send_goal(new_goal, send_goal_options);
  }

  void feedback_callback(GoalHandle::SharedPtr, const std::shared_ptr<const nav2_msgs::action::FollowWaypoints::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Current waypoint: %d", feedback->current_waypoint);
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
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinit_client_;
  rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_costmap_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;

  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr client_ptr_;

  rclcpp::Rate loop_rate_;
  bool first_callback_ = true;
  int mode_;
  float freq_;
  float goal_x_, goal_y_, goal_theta_;
  double cov_x_, cov_y_;
  float cov_tol_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavigationGoals>());
  rclcpp::shutdown();
  return 0;
}