#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cinttypes>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"
using namespace std::chrono_literals;

class LocalizationHoming : public rclcpp::Node
{
public:
  LocalizationHoming() : Node("homing_node"), loop_rate_(10.0f), freq_(10.0f), cov_tol_(0.1)
  {
    rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true)); // Set to false for real robot
    set_parameter(simTime);

    reinit_client_ = this->create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
    clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>("/global_costmap/clear_around_global_costmap");
    pub_init_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 1, std::bind(&LocalizationHoming::pose_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MAKE SURE THAT ROBOT HAS AN EMPTY 1mX1m AREA IN FRONT OF IT");
    RCLCPP_INFO(this->get_logger(), "Select mode: \n"
                                    "0 - reset localization and do homing sequence \n"
                                    "1 - set manual localization and do homing sequence \n"
                                    "2 - no homing sequence start with localization 0.0 \n");

    std::cin >> mode_;
    switch (mode_)
    {
    case 0:
      reset_amcl();
      break;
    case 1:
      init_pose();
      break;
    default:
      break;
    }

    do_homing_sequence();

    if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
    {
      RCLCPP_INFO(this->get_logger(), "High covariance");
      RCLCPP_INFO(this->get_logger(), "Use teleop_twist_keyboard to drive manually and improve localization");
    }
    RCLCPP_INFO(this->get_logger(), "Homing done");
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
    init_pose.pose.covariance[14] = 0.625;
    RCLCPP_INFO(this->get_logger(), "Setting pose to x=%.2f y=%.2f theta=%.2f", init_pose.pose.pose.position.x,
                init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z);

    pub_init_pose_->publish(init_pose);
    rclcpp::spin_some(this->get_node_base_interface());
  }

  void do_homing_sequence()
  {
    // TODO use odometry
    // simple controls without checking for odometry
    float vel = 0.3f;
    float omega = 0.3f;
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
    clear_costmap();

    rclcpp::spin_some(this->get_node_base_interface());
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
      clear_costmap();
      loop_rate_.sleep();
    }

    rclcpp::spin_some(this->get_node_base_interface());
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
    rclcpp::spin_some(this->get_node_base_interface());
  }

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinit_client_;
  rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_costmap_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;

  rclcpp::Rate loop_rate_;
  int mode_;
  float freq_;
  double cov_x_, cov_y_;
  float cov_tol_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationHoming>());
  rclcpp::shutdown();
  return 0;
}