#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class RoomMappingRobot : public rclcpp::Node {
public:
    RoomMappingRobot() : Node("room_mapping_robot"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        laser_scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&RoomMappingRobot::laserScanCallback, this, std::placeholders::_1));
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&RoomMappingRobot::odomCallback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        map_.header.frame_id = "map";
        map_.info.resolution = 0.05; // 5 cm resolution
        map_.info.width = 200;       // 10 meters wide
        map_.info.height = 200;      // 10 meters high
        map_.info.origin.position.x = -10.0;
        map_.info.origin.position.y = -10.0;
        map_.data.resize(map_.info.width * map_.info.height, -1); // Initialize to unknown

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&RoomMappingRobot::updateMap, this));
    }

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        latest_scan_ = *msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;

        geometry_msgs::msg::TransformStamped transformStamped;
        try {
            transformStamped = tf_buffer_.lookupTransform("map", "base_link", this->get_clock()->now());
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
            return;
        }
        robot_pose_ = transformStamped;

    }

    void updateMap() {
        if (latest_scan_.ranges.empty() || !robot_pose_.header.frame_id.empty()) return;

        double robot_x = robot_pose_.transform.translation.x;
        double robot_y = robot_pose_.transform.translation.y;
        tf2::Quaternion q(robot_pose_.transform.rotation.x, robot_pose_.transform.rotation.y, robot_pose_.transform.rotation.z, robot_pose_.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
            float range = latest_scan_.ranges[i];
            if (range < latest_scan_.range_min || range > latest_scan_.range_max || std::isnan(range) || std::isinf(range)) continue;

            double angle = yaw + latest_scan_.angle_min + i * latest_scan_.angle_increment;
            double obstacle_x = robot_x + range * cos(angle);
            double obstacle_y = robot_y + range * sin(angle);

            int map_x = static_cast<int>((obstacle_x - map_.info.origin.position.x) / map_.info.resolution);
            int map_y = static_cast<int>((obstacle_y - map_.info.origin.position.y) / map_.info.resolution);

            if (map_x >= 0 && map_x < map_.info.width && map_y >= 0 && map_y < map_.info.height) {
                map_.data[map_y * map_.info.width + map_x] = 100; // Occupied
            }

            // Free space update
            for (float r = 0; r < range; r += map_.info.resolution) {
                double free_x = robot_x + r * cos(angle);
                double free_y = robot_y + r * sin(angle);
                int free_map_x = static_cast<int>((free_x - map_.info.origin.position.x) / map_.info.resolution);
                int free_map_y = static_cast<int>((free_y - map_.info.origin.position.y) / map_.info.resolution);

                if (free_map_x >= 0 && free_map_x < map_.info.width && free_map_y >= 0 && free_map_y < map_.info.height) {
                    if (map_.data[free_map_y * map_.info.width + free_map_x] != 100) {
                        map_.data[free_map_y * map_.info.width + free_map_x] = 0; // Free
                    }
                }
            }
        }
        map_.header.stamp = this->get_clock()->now();
        map_publisher_->publish(map_);

        // Simple exploration (replace with advanced navigation)
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = 0.1;
        cmd_vel_publisher_->publish(cmd_vel);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    sensor_msgs::msg::LaserScan latest_scan_;
    nav_msgs::msg::Odometry latest_odom_;
    geometry_msgs::msg::TransformStamped robot_pose_;
    nav_msgs::msg::OccupancyGrid map_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoomMappingRobot>());
    rclcpp::shutdown();
    return 0;
}
