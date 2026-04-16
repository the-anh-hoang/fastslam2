#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> 
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cstdint>
#include "particle_filter_localizer/particle_filter.hpp"
class ParticleFilterNode : public rclcpp::Node 
{
    public: 
    ParticleFilterNode() : Node("particle_filter_node") {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map",
            rclcpp::QoS(10).transient_local(),
            std::bind(&ParticleFilterNode::callback, this, std::placeholders::_1)
        );

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10, 
            std::bind(&ParticleFilterNode::odomCallback, this, std::placeholders::_1)
        );

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&ParticleFilterNode::scanCallback, this, std::placeholders::_1)
        ); 

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        map_to_odom_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/particles", 10
        );

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&ParticleFilterNode::publishParticles, this)
        );
    }

    private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    geometry_msgs::msg::TransformStamped t;
    std::unique_ptr<ParticleFilter> pf_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> map_to_odom_broadcaster_;
    bool map_received = false;
    rclcpp::TimerBase::SharedPtr timer_; 

    // odom sub
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void callback(nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        if (map_received) return;
        RCLCPP_INFO(this->get_logger(), "Map received; Width: %d, Height %d", map->info.width, map->info.height);
        map_received = true;
        map_ = map;

        // init particle filter
        pf_ = std::make_unique<ParticleFilter>(500, this);
        pf_->initialize(*map);

    }

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // RCLCPP_INFO(this->get_logger(), "odom received: x: %lf, y: %lf", odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y);
        if (!pf_) return;
        pf_->predict(*odom_msg);

    }

    void scanCallback(sensor_msgs::msg::LaserScan::SharedPtr scan) {
        if (!pf_) return;
        try {
            t = tf_buffer_->lookupTransform("base_footprint", "base_scan", tf2::TimePointZero, std::chrono::milliseconds(100));
            pf_->update(*scan, t);
            pf_->publishDebugMarkers(*scan, t);

            t = tf_buffer_->lookupTransform("odom" , "base_footprint", tf2::TimePointZero, std::chrono::milliseconds(100)); 
            std::vector<double> transform = pf_->caclBaseToOdomTransform(t);
            
            geometry_msgs::msg::TransformStamped map_odom;
            map_odom.header.stamp = this->now();
            map_odom.header.frame_id = "map";
            map_odom.child_frame_id = "odom";
            map_odom.transform.translation.x = transform[0];
            map_odom.transform.translation.y = transform[1];
            map_odom.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0,0,transform[2]);
            map_odom.transform.rotation.x = q.x(); 
            map_odom.transform.rotation.y = q.y();  
            map_odom.transform.rotation.z = q.z();  
            map_odom.transform.rotation.w = q.w();  
            map_to_odom_broadcaster_->sendTransform(map_odom); 
            
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform source_frame to target_frame: %s", ex.what());
        }
    }
    
    void publishParticles() {
        // helper to visualize particles
        if (!pf_) return;
        geometry_msgs::msg::PoseArray msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        for (const auto& p : pf_->getParticles()) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = p.x;
            pose.position.y = p.y;
            pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, p.theta);
            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();
            
            msg.poses.push_back(pose);
            
        }
        particle_pub_->publish(msg);
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParticleFilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0; 
}