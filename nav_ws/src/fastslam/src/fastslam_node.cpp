#include <vector> 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> 
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/header.hpp> 
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "fastslam/occupancy_grid_map.hpp"
#include "fastslam/scan_integrator.hpp"
#include "fastslam/particle.hpp"
#include "fastslam/motion_model.hpp"



namespace fastslam {
    class FastSlamNode : public rclcpp::Node {
        public:
        FastSlamNode() : 
        Node("fast_slam_node"),
        md_(a1_, a2_, a3_, a4_),
        particles_(num_particles_, Particle()) {
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(&FastSlamNode::odomCallback, this, std::placeholders::_1)
            );
            particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
                "/particles", 10
            );

        }


        private:
        // CONFIGS for now 
        int num_particles_ = 10; 
        double a1_=0.1, a2_=0.1, a3_=0.1, a4_=0.1;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        Pose prev_pose_;
        bool initialized_ = false;
        MotionModel md_; 
        std::vector<Particle> particles_; 
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

        void odomCallback(const nav_msgs::msg::Odometry& odom_msg) {

            double curr_x = odom_msg.pose.pose.position.x;
            double curr_y = odom_msg.pose.pose.position.y;
            double curr_theta = getYaw(odom_msg); 
            Pose curr_pose(curr_x, curr_y, curr_theta); 

            if (!initialized_) {
                prev_pose_ = Pose(curr_x, curr_y, curr_theta);
                initialized_ = true;
                return;
            }
            double dx = curr_x - prev_pose_.x;
            double dy = curr_y - prev_pose_.y;
            double delta_dist = std::sqrt(dx*dx + dy*dy);
            double delta_rot = std::abs(curr_theta - prev_pose_.theta);

            if (delta_dist < 0.1 && delta_rot < 0.05) return;

            Pose sampled_pose;
            for (Particle& p : particles_) {
                sampled_pose = md_.applyMotionModel(
                    Pose(p.x, p.y, p.theta),
                    prev_pose_,
                    curr_pose);
                p.x = sampled_pose.x;
                p.y = sampled_pose.y;
                p.theta = sampled_pose.theta; 
            }
            prev_pose_ = curr_pose; 
            publishParticles(); 
        }

        double getYaw(const nav_msgs::msg::Odometry& odom_msg) {
            double qx = odom_msg.pose.pose.orientation.x;
            double qy = odom_msg.pose.pose.orientation.y;
            double qz = odom_msg.pose.pose.orientation.z;
            double qw = odom_msg.pose.pose.orientation.w;
            return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        }

        void publishParticles() {
            // helper to visualize particles
            geometry_msgs::msg::PoseArray msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            for (const Particle& p : particles_) {
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
            particles_pub_->publish(msg);
        }


    };


} // namespace fastslam

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<fastslam::FastSlamNode>());
    rclcpp::shutdown();
    return 0;
}