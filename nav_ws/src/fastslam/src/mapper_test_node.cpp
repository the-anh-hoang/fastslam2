#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/header.hpp> 
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "fastslam/occupancy_grid_map.hpp"
#include "fastslam/scan_integrator.hpp"


namespace fastslam {

class MapperTestNode : public rclcpp::Node {
public:
    MapperTestNode() : Node("mapper_test_node")
    , map_(fastslam::MapParams{})
    , integrator_(0.7f, -0.7f, 1, 0.0, 0.0, 0.0)  // zero laser offset
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&MapperTestNode::scanCallback, this, std::placeholders::_1)
        );
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MapperTestNode::publishMap, this)
        );
    }

private:
    double prev_x_ = 0.0;
    double prev_y_ = 0.0;
    double prev_theta_ = 0.0;
    bool initialized_ = false;
    OccupancyGridMap map_;
    ScanIntegrator integrator_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    void scanCallback(const sensor_msgs::msg::LaserScan& scan) {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                "map", "base_footprint", scan.header.stamp
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transfrom %s to %s: %s",
                "map", "footprint", ex.what()
            );
        }
        if (!initialized_) {
            prev_x_ = t.transform.translation.x;
            prev_y_ = t.transform.translation.y;
            prev_theta_ = getYaw(t);
            integrator_.integrateScan(map_, scan, prev_x_, prev_y_, prev_theta_);
            initialized_ = true; 
        } else {
            double robot_x = t.transform.translation.x;
            double robot_y = t.transform.translation.y;
            double robot_theta = getYaw(t);
            double dx = robot_x - prev_x_;
            double dy = robot_y - prev_y_;
            double delta_dist = std::sqrt(dx*dx + dy*dy);
            double delta_angle = std::abs(robot_theta - prev_theta_);

            // if (delta_dist < 0.5 && delta_angle < 0.436) return;
            if (delta_dist < 1) return;

            integrator_.integrateScan(map_, scan, robot_x, robot_y, robot_theta);
            prev_x_ = robot_x;
            prev_y_ = robot_y;
            prev_theta_ = robot_theta;
        }
        
        
    }

    void publishMap() {
        auto map_msg = nav_msgs::msg::OccupancyGrid(); 
        auto header = std_msgs::msg::Header();
        auto map_meta_data = nav_msgs::msg::MapMetaData(); 
        header.stamp = this->now();
        header.frame_id = "map";
        fastslam::MapParams map_params = map_.getMapParams(); 
        map_meta_data.map_load_time.sec = 0;
        map_meta_data.map_load_time.nanosec = 0;
        map_meta_data.resolution = map_params.resolution;
        map_meta_data.width = map_params.width;
        map_meta_data.height = map_params.height;
        map_meta_data.origin.position.x = map_params.origin_x;
        map_meta_data.origin.position.y = map_params.origin_y;
        map_meta_data.origin.position.z = 0.0;
        map_meta_data.origin.orientation.x = 0; 
        map_meta_data.origin.orientation.y = 0; 
        map_meta_data.origin.orientation.z = 0; 
        map_meta_data.origin.orientation.w = 1;

        map_msg.header = header; 
        map_msg.info = map_meta_data;
        map_msg.data = map_.toROSData();
        map_pub_->publish(map_msg); 
    }

    double getYaw(geometry_msgs::msg::TransformStamped& t) {
        double qx = t.transform.rotation.x;
        double qy = t.transform.rotation.y;
        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;
        return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
};
}
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fastslam::MapperTestNode>());
    rclcpp::shutdown();
    return 0;
}