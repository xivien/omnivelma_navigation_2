#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher()
        : Node("minimal_publisher"), min_ang_(-3.14), max_ang_(3.14), range_min_(0.45), range_max_(25.0), frame_id_("base_laser")
    {
        tfListener = new tf2_ros::TransformListener(tf2_buffer);

        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&MinimalPublisher::timer_callback, this));
        subscription_1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/monokl_l/scan", 10, std::bind(&MinimalPublisher::topic_1_callback, this, std::placeholders::_1));
        subscription_2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/monokl_r/scan", 10, std::bind(&MinimalPublisher::topic_2_callback, this, std::placeholders::_1));
    }

private:
    void timer_callback()
    {
        if (!tf2_buffer._frameExists("base_laser"))
        {
            RCLCPP_INFO(this->get_logger(), "base_footprint doesn't exist");
            return;
        }
        if (data_1.header.frame_id == "")
        {
            RCLCPP_INFO(this->get_logger(), "Empty frame - %s - id ", data_1.header.frame_id.c_str());
            return;
        }

        //change laserscans to pointcloud
        sensor_msgs::msg::PointCloud2 tmpCloud1, tmpCloud2, merged_cloud;

        projector_.transformLaserScanToPointCloud(
            frame_id_, data_1, tmpCloud1, tf2_buffer, -1.0,
            laser_geometry::channel_option::Distance);

        projector_.transformLaserScanToPointCloud(
            frame_id_, data_2, tmpCloud2, tf2_buffer, -1.0,
            laser_geometry::channel_option::Distance);
        // concatenate clouds
        pcl::concatenatePointCloud(tmpCloud1, tmpCloud2, merged_cloud);

        // build laserscan output
        auto scan_msg = sensor_msgs::msg::LaserScan();

        scan_msg.header = data_1.header;
        scan_msg.header.frame_id = frame_id_;
        scan_msg.angle_min = min_ang_;
        scan_msg.angle_max = max_ang_;
        scan_msg.angle_increment = 0.0058;
        scan_msg.scan_time = 0.0333333;
        scan_msg.range_min = range_min_;
        scan_msg.range_max = range_max_;

        uint32_t ranges_size = std::ceil((max_ang_ - min_ang_) / scan_msg.angle_increment) + 1; // TODO Fixed SLAM
        scan_msg.ranges.assign(ranges_size, range_max_ + 1.0);

        const double range_min_sq_ = range_min_ * range_min_;
        for (sensor_msgs::PointCloud2ConstIterator<float> it(merged_cloud, "x"); it != it.end(); ++it)
        {
            // TODO: do something with the values of x, y, z
            float x = it[0];
            float y = it[1];
            float z = it[2];

            if (std::isnan(x) || std::isnan(y) || std::isnan(z))
            {
                continue;
            }
            double range_sq = y * y + x * x;

            if (range_sq < range_min_sq_)
            {
                RCLCPP_INFO(this->get_logger(), "rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
                continue;
            }

            double angle = atan2(y, x);
            // if (angle < min_ang_ || angle > max_ang_)
            // {
            //     RCLCPP_INFO(this->get_logger(), "rejected for angle not in range.");
            //     continue;
            // }
            int index = (angle - min_ang_) / scan_msg.angle_increment;

            if (scan_msg.ranges[index] * scan_msg.ranges[index] > range_sq)
                scan_msg.ranges[index] = sqrt(range_sq);
        }

        publisher_->publish(scan_msg);
    }

    void topic_1_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        data_1 = *msg;
    }
    void topic_2_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        data_2 = *msg;
    }

    float min_ang_, max_ang_, range_min_, range_max_;
    std::string frame_id_;

    tf2::BufferCore tf2_buffer;
    tf2_ros::TransformListener *tfListener;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::msg::LaserScan data_1; //sharedPtr
    sensor_msgs::msg::LaserScan data_2;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_1_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_2_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}