#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/header.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "fastslam/fastslam2.hpp"
#include "fastslam/laser_scan.hpp"
#include "fastslam/pose.hpp"



namespace fastslam {
    // ROS2 plumbing around the pure-C++ FastSlam2 engine: topics in,
    // topics/TF out. All filter math lives in fastslam/fastslam2.hpp.
    class FastSlam2Node : public rclcpp::Node {
        public:
        FastSlam2Node() : Node("fastslam2_node") {
            // params
            this->declare_parameter("num_particles", 3);
            this->declare_parameter("map_chunk_size", 24.0f);
            this->declare_parameter("map_res", 0.03f); // (m/cell)
            this->declare_parameter("a1", 0.01);
            this->declare_parameter("a2", 0.01);
            this->declare_parameter("a3", 0.01);
            this->declare_parameter("a4", 0.01);
            this->declare_parameter("scan_match_x_range", 0.09);
            this->declare_parameter("scan_match_y_range", 0.09);
            this->declare_parameter("scan_match_theta_range", M_PI/4); // quite excessive
            this->declare_parameter("scan_match_step_xy", 0.03);
            this->declare_parameter("scan_match_step_theta", 0.02);
            this->declare_parameter("ray_skip", 5);
            this->declare_parameter("z_hit", 0.8);
            this->declare_parameter("std_hit", 0.2);
            this->declare_parameter("z_rand", 0.2);
            this->declare_parameter("linear_update", 0.5);
            this->declare_parameter("angular_update", 0.2);
            this->declare_parameter("resample_threshold", 0.5);
            this->declare_parameter("map_publish_period", 5.0);  // seconds between /map publishes
            this->declare_parameter("traj_output_dir", "results");  // TUM trajectories written here on shutdown

            FastSlam2Config config;
            config.num_particles = this->get_parameter("num_particles").as_int();
            config.map_chunk_size = static_cast<float>(this->get_parameter("map_chunk_size").as_double());
            config.map_res = static_cast<float>(this->get_parameter("map_res").as_double());
            config.a1 = this->get_parameter("a1").as_double();
            config.a2 = this->get_parameter("a2").as_double();
            config.a3 = this->get_parameter("a3").as_double();
            config.a4 = this->get_parameter("a4").as_double();
            config.ray_skip = this->get_parameter("ray_skip").as_int();
            config.scan_match_x_range = this->get_parameter("scan_match_x_range").as_double();
            config.scan_match_y_range = this->get_parameter("scan_match_y_range").as_double();
            config.scan_match_theta_range = this->get_parameter("scan_match_theta_range").as_double();
            config.scan_match_step_xy = this->get_parameter("scan_match_step_xy").as_double();
            config.scan_match_step_theta = this->get_parameter("scan_match_step_theta").as_double();
            config.z_hit = this->get_parameter("z_hit").as_double();
            config.std_hit = this->get_parameter("std_hit").as_double();
            config.z_rand = this->get_parameter("z_rand").as_double();
            config.linear_update = this->get_parameter("linear_update").as_double();
            config.angular_update = this->get_parameter("angular_update").as_double();
            config.resample_threshold = this->get_parameter("resample_threshold").as_double();

            map_publish_period_ = this->get_parameter("map_publish_period").as_double();
            traj_output_dir_ = this->get_parameter("traj_output_dir").as_string();
            last_map_pub_time_ = this->now();

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            try {
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    "base_footprint",
                    "base_scan",
                    tf2::TimePointZero,
                    std::chrono::seconds(3)
                );

                config.laser_dx = t.transform.translation.x;
                config.laser_dy = t.transform.translation.y;
                config.laser_dtheta = quatToYaw(
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w
                );
            } catch (const tf2::TransformException& ex) {
                RCLCPP_FATAL(this->get_logger(), "FAILED TO GET LASER TF: %s", ex.what());
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Laser TF: dx=%.4f dy=%.4f dtheta=%.4f",
                            config.laser_dx, config.laser_dy, config.laser_dtheta);
            RCLCPP_INFO(this->get_logger(), "num_particles: %d", config.num_particles);
            RCLCPP_INFO(this->get_logger(), "linear_update: %.2f", config.linear_update);

            slam_ = std::make_unique<FastSlam2>(config);

            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&FastSlam2Node::scanCallback, this, std::placeholders::_1)
            );
            auto map_qos = rclcpp::QoS(1).transient_local().reliable();
            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
            particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
                "/particles", 10
            );
            map_odom_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Initialize transform 0 0 0
            map_odom_tf_.header.stamp = this->now();
            map_odom_tf_.header.frame_id = "map";
            map_odom_tf_.child_frame_id = "odom";
            map_odom_tf_.transform.translation.x = 0.0;
            map_odom_tf_.transform.translation.y = 0.0;
            map_odom_tf_.transform.translation.z = 0.0;
            map_odom_tf_.transform.rotation.x = 0.0;
            map_odom_tf_.transform.rotation.y = 0.0;
            map_odom_tf_.transform.rotation.z = 0.0;
            map_odom_tf_.transform.rotation.w = 1.0;
            publishMapToOdom();

            tf_pub_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(20), // 50hz
                std::bind(&FastSlam2Node::publishMapToOdom, this)

            );

        }

        ~FastSlam2Node() override {
            if (slam_) {
                slam_->writeTrajectories(traj_output_dir_);
                RCLCPP_INFO(this->get_logger(), "Wrote trajectories to %s", traj_output_dir_.c_str());
            }
        }


        private:

        std::unique_ptr<FastSlam2> slam_;

        // Map publish throttle
        double map_publish_period_;
        rclcpp::Time last_map_pub_time_;

        std::string traj_output_dir_;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> map_odom_pub_;
        geometry_msgs::msg::TransformStamped map_odom_tf_;
        rclcpp::TimerBase::SharedPtr tf_pub_timer_;

        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;


        bool getRobotOdomPose(const builtin_interfaces::msg::Time& stamp, Pose& out) {
            geometry_msgs::msg::TransformStamped t;
            try {
                t = tf_buffer_->lookupTransform(
                    "odom", "base_footprint",
                    tf2_ros::fromMsg(stamp), std::chrono::milliseconds(100));
            } catch (const tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(),
                    "No odom->base_footprint at scan time, skipping scan: %s", ex.what());
                return false;
            }
            out.x = t.transform.translation.x;
            out.y = t.transform.translation.y;
            out.theta = quatToYaw(
                t.transform.rotation.x, t.transform.rotation.y,
                t.transform.rotation.z, t.transform.rotation.w);
            return true;
        }

        static LaserScan toEngineScan(const sensor_msgs::msg::LaserScan& msg) {
            LaserScan scan;
            scan.range_min = msg.range_min;
            scan.range_max = msg.range_max;
            scan.angle_min = msg.angle_min;
            scan.angle_increment = msg.angle_increment;
            scan.ranges = msg.ranges;
            return scan;
        }

        void scanCallback(const sensor_msgs::msg::LaserScan& scan_msg) {
            // Pull the robot's odom pose at the SCAN's timestamp
            Pose curr_pose;
            if (!getRobotOdomPose(scan_msg.header.stamp, curr_pose)) return;

            UpdateResult res = slam_->processScan(
                toEngineScan(scan_msg),
                curr_pose,
                rclcpp::Time(scan_msg.header.stamp).seconds()
            );

            switch (res.status) {
                case UpdateResult::Status::Skipped:
                    return;
                case UpdateResult::Status::Initialized:
                    publishMapToOdom();
                    publishMap(slam_->bestParticle());
                    return;
                case UpdateResult::Status::Updated:
                    break;
            }

            calculateOdomTf(res.best_pose);
            if ((this->now() - last_map_pub_time_).seconds() >= map_publish_period_) {
                publishMap(slam_->bestParticle());
                last_map_pub_time_ = this->now();
            }
            publishParticles();

            // === LOGGING ===
            RCLCPP_INFO(this->get_logger(),
                "[%d] %.1fms | dist=%.1fm | N_eff=%.1f/%zu (%.0f%%) | spread=%.4fm | %s",
                res.scan_count, res.duration_ms, res.total_distance,
                res.n_eff, slam_->particles().size(), res.n_eff_ratio * 100.0,
                res.spread,
                res.did_resample ? "RESAMPLED" : "");
            RCLCPP_INFO(this->get_logger(),
                "  scan_match: ll=[%.1f, %.1f, %.1f] | proposal_cov: σx=%.5f σy=%.5f σθ=%.5f",
                res.min_ll, res.avg_ll, res.max_ll,
                std::sqrt(res.avg_cov_xx), std::sqrt(res.avg_cov_yy), std::sqrt(res.avg_cov_tt));
            RCLCPP_INFO(this->get_logger(),
                "  weights: [%.4f, %.4f] | best=(%.2f,%.2f,%.2f) | resamples=%d/%d",
                res.w_min, res.w_max,
                res.best_pose.x, res.best_pose.y, res.best_pose.theta,
                res.resample_count, res.scan_count);
        }


        void calculateOdomTf(const Pose& map_to_base) {
            geometry_msgs::msg::TransformStamped base_to_odom = tf_buffer_->lookupTransform(
                "base_footprint",
                "odom",
                tf2::TimePointZero,
                std::chrono::milliseconds(100)
            );
            double map_to_base_x = map_to_base.x;
            double map_to_base_y = map_to_base.y;
            double map_to_base_rot = map_to_base.theta;

            double base_to_odom_x = base_to_odom.transform.translation.x;
            double base_to_odom_y = base_to_odom.transform.translation.y;
            double base_to_odom_rot = quatToYaw(
                base_to_odom.transform.rotation.x,
                base_to_odom.transform.rotation.y,
                base_to_odom.transform.rotation.z,
                base_to_odom.transform.rotation.w
            );

            double s = std::sin(map_to_base_rot);
            double c = std::cos(map_to_base_rot);
            double map_to_odom_x = map_to_base_x + c*base_to_odom_x - s*base_to_odom_y;
            double map_to_odom_y = map_to_base_y + s*base_to_odom_x + c*base_to_odom_y;
            double map_to_odom_rot = map_to_base_rot + base_to_odom_rot;
            map_odom_tf_.transform.translation.x = map_to_odom_x;
            map_odom_tf_.transform.translation.y = map_to_odom_y;
            map_odom_tf_.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0,0,map_to_odom_rot);
            map_odom_tf_.transform.rotation.x = q.x();
            map_odom_tf_.transform.rotation.y = q.y();
            map_odom_tf_.transform.rotation.z = q.z();
            map_odom_tf_.transform.rotation.w = q.w();
        }

        void publishMapToOdom() {
            map_odom_tf_.header.stamp = this->now();
            map_odom_pub_->sendTransform(map_odom_tf_);
        }

        void publishMap(const Particle& p) {
            auto map_msg = nav_msgs::msg::OccupancyGrid();
            auto header = std_msgs::msg::Header();
            auto map_meta_data = nav_msgs::msg::MapMetaData();
            header.stamp = this->now();
            header.frame_id = "map";
            MapParams map_params = p.map.getMapParams();
            GridData grid = p.map.toGridData();
            map_meta_data.map_load_time.sec = 0;
            map_meta_data.map_load_time.nanosec = 0;
            map_meta_data.resolution = map_params.resolution;
            map_meta_data.width = grid.width;
            map_meta_data.height = grid.height;
            map_meta_data.origin.position.x = grid.origin_x;
            map_meta_data.origin.position.y = grid.origin_y;
            map_meta_data.origin.position.z = 0.0;
            map_meta_data.origin.orientation.x = 0;
            map_meta_data.origin.orientation.y = 0;
            map_meta_data.origin.orientation.z = 0;
            map_meta_data.origin.orientation.w = 1;

            map_msg.header = header;
            map_msg.info = map_meta_data;
            map_msg.data = grid.data;
            map_pub_->publish(map_msg);
        }

        double quatToYaw(double qx, double qy, double qz, double qw) {
            return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        }

        void publishParticles() {
            geometry_msgs::msg::PoseArray msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            for (const Particle& p : slam_->particles()) {
                geometry_msgs::msg::Pose pose;
                pose.position.x = p.poses.back().x;
                pose.position.y = p.poses.back().y;
                pose.position.z = 0.0;
                tf2::Quaternion q;
                q.setRPY(0, 0, p.poses.back().theta);
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
    rclcpp::spin(std::make_shared<fastslam::FastSlam2Node>());
    rclcpp::shutdown();
    return 0;
}
