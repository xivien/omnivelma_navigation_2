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

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class LocalizationHoming : public rclcpp::Node
{
public:
  LocalizationHoming() : Node("homing_node"), loop_rate_(10.0f), freq_(10.0f), cov_tol_(0.2)
  {
    rclcpp::Parameter simTime("use_sim_time", rclcpp::ParameterValue(true)); // Set to false for real robot
    set_parameter(simTime);

    reinit_client_ = this->create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
    clear_costmap_client_ = this->create_client<nav2_msgs::srv::ClearCostmapAroundRobot>("/global_costmap/clear_around_global_costmap");
    pub_init_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1);
    pub_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    amcl_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 1, std::bind(&LocalizationHoming::pose_callback, this, std::placeholders::_1));
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LocalizationHoming::odom_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "MAKE SURE THAT ROBOT HAS AN EMPTY 1mX1m AREA IN FRONT OF IT");
    RCLCPP_INFO(this->get_logger(), "Select mode: \n"
                                    "0 - reset localization and do homing sequence \n"
                                    "1 - set manual localization and do homing sequence \n");

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
    RCLCPP_INFO(this->get_logger(), "Homing done, localization quality is good");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (first_callback_)
    {
      first_callback_ = false;
      start_pos_x_ = msg->pose.pose.position.x;
      start_pos_y_ = msg->pose.pose.position.y;
    }
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
    cov_x_ = msg->pose.covariance[0];
    cov_y_ = msg->pose.covariance[7];
  }

  void clear_costmap()
  {
    auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
    int counter = 0;
    while (!clear_costmap_client_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      counter++;
      if (counter > 5){
        RCLCPP_INFO(this->get_logger(), "service not available, ending wait...");
        break;
      }
    }
    auto result = clear_costmap_client_->async_send_request(request);

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
      RCLCPP_INFO(this->get_logger(), "Succeeded calling service");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }
  }

  void init_pose()
  {
    geometry_msgs::msg::PoseWithCovarianceStamped init_pose;
    init_pose.header.frame_id = "map";
    init_pose.pose.pose.orientation.w = 1;
    RCLCPP_INFO(this->get_logger(), "Enter X[m] Y[m] Theta[rad]");
    tf2::Quaternion myQuaternion;
    double yaw;
    std::cin >> init_pose.pose.pose.position.x >> init_pose.pose.pose.position.y >> yaw;
    myQuaternion.setRPY( 0.0f, 0.0f, yaw );
    init_pose.pose.pose.orientation.x = myQuaternion.x();
    init_pose.pose.pose.orientation.y = myQuaternion.y();
    init_pose.pose.pose.orientation.z = myQuaternion.z();
    init_pose.pose.pose.orientation.w = myQuaternion.w();
    // init covariances
    init_pose.pose.covariance[0] = 10.0;
    init_pose.pose.covariance[7] = 10.0;
    init_pose.pose.covariance[14] = 3.0;
    RCLCPP_INFO(this->get_logger(), "Setting pose to x=%.2f y=%.2f theta=%.2f", init_pose.pose.pose.position.x,
                init_pose.pose.pose.position.y, yaw);

    pub_init_pose_->publish(init_pose);
    rclcpp::spin_some(this->get_node_base_interface());
  }

  void do_square()
  {
    std::array<float, 4> destination_x = {2.0f + start_pos_x_, 2.0f + start_pos_x_, 0.0f + start_pos_x_, 0.0f + start_pos_x_};
    std::array<float, 4> destination_y = {0.0f + start_pos_y_, -2.0f + start_pos_y_, -2.0f + start_pos_y_, 0.0f + start_pos_y_};
    geometry_msgs::msg::Twist vel_msg;

    for (size_t i = 0; i < 4; i++)
    {
      float last_direct = 10000.0f;
      float last_dist = 10000.0f;

      float dx, dy;
      dx = destination_x[i] - pos_x_;
      dy = destination_y[i] - pos_y_;

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
      dx = destination_x[i] - pos_x_;
      dy = destination_y[i] - pos_y_;
      float dist = std::sqrt(dx * dx + dy * dy);

      while (std::abs(dist) <= std::abs(last_dist))
      {
        vel_msg.linear.x = vel_;
        pub_vel_->publish(vel_msg);
        loop_rate_.sleep();
        rclcpp::spin_some(this->get_node_base_interface());

        last_dist = dist;
        dx = destination_x[i] - pos_x_;
        dy = destination_y[i] - pos_y_;
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
      if (std::abs(direct) < 0.1f && std::abs(direct) > std::abs(last_direct))
      {
        break;
      }
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
    }
    vel_msg.angular.z = 0.0f;
    pub_vel_->publish(vel_msg);
    loop_rate_.sleep();
    rclcpp::spin_some(this->get_node_base_interface());
  }

  void do_homing_sequence()
  {
    rclcpp::sleep_for(1s);
    rclcpp::spin_some(this->get_node_base_interface());
    if (cov_x_ > cov_tol_ || cov_y_ > cov_tol_)
    {
      RCLCPP_INFO(this->get_logger(), "High covariance, doing a square 2x2m");
      do_square();
      clear_costmap();
      loop_rate_.sleep();
    }
    rclcpp::sleep_for(1s);
    rclcpp::spin_some(this->get_node_base_interface());
  }

  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reinit_client_;
  rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr clear_costmap_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_pose_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  nav_msgs::msg::Odometry::SharedPtr odom_;

  bool first_callback_ = true;
  rclcpp::Rate loop_rate_;
  int mode_;
  float freq_;
  double cov_x_, cov_y_;
  float cov_tol_;
  double yaw_, pos_x_, pos_y_;
  double start_pos_x_, start_pos_y_;
  float vel_ = 0.3f;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationHoming>());
  rclcpp::shutdown();
  return 0;
}