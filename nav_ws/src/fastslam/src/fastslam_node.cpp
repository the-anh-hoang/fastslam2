#include <vector> 
#include <random> 

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

#include "fastslam/occupancy_grid_map.hpp"
#include "fastslam/scan_integrator.hpp"
#include "fastslam/particle.hpp"
#include "fastslam/motion_model.hpp"
#include "fastslam/pose.hpp"



namespace fastslam {
    class FastSlamNode : public rclcpp::Node {
        public:
        FastSlamNode() : Node("fastslam_node"), gen_(rd_()) {
            // params 
            this->declare_parameter("num_particles", 20);
            this->declare_parameter("map_width", 1000); 
            this->declare_parameter("map_height", 1000);
            this->declare_parameter("map_res", 0.03f); // (m/cell)
            this->declare_parameter("a1", 0.01);
            this->declare_parameter("a2", 0.01);
            this->declare_parameter("a3", 0.01);
            this->declare_parameter("a4", 0.01);
            this->declare_parameter("z_hit", 0.90);
            this->declare_parameter("std_hit", 0.2);
            this->declare_parameter("z_rand", 0.1);
            this->declare_parameter("linear_update", 0.5);
            this->declare_parameter("angular_update", 0.2);
            this->declare_parameter("resample_threshold", 0.5);
            
            num_particles_ = this->get_parameter("num_particles").as_int();
            int map_width = this->get_parameter("map_width").as_int();
            int map_height = this->get_parameter("map_height").as_int();
            float map_res = static_cast<float>(this->get_parameter("map_res").as_double()); 
            MapParams mp(map_width, map_height, map_res); 
            a1_ = this->get_parameter("a1").as_double();
            a2_ = this->get_parameter("a2").as_double();            
            a3_ = this->get_parameter("a3").as_double();
            a4_ = this->get_parameter("a4").as_double();
            z_hit_ = this->get_parameter("z_hit").as_double(); 
            std_hit_ = this->get_parameter("std_hit").as_double();
            z_rand_ = this->get_parameter("z_rand").as_double();
            linear_update_ = this->get_parameter("linear_update").as_double();
            angular_update_ = this->get_parameter("angular_update").as_double();
            resample_threshold_ = this->get_parameter("resample_threshold").as_double(); 
            RCLCPP_INFO(this->get_logger(), "num_particles: %d", num_particles_);
            RCLCPP_INFO(this->get_logger(), "a1: %.4f", a1_);
            RCLCPP_INFO(this->get_logger(), "linear_update: %.2f", linear_update_);

            md_ = MotionModel(a1_, a2_, a3_, a4_);
            particles_ = std::vector<Particle>(num_particles_, Particle(mp)); 
            integrator_ = ScanIntegrator(0.7f, -0.7f, 1, -0.064, 0.0, 0.0, 1);

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(&FastSlamNode::odomCallback, this, std::placeholders::_1)
            );
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&FastSlamNode::scanCallback, this, std::placeholders::_1)
            ); 

            map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
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
                std::chrono::milliseconds(16), // 20hz
                std::bind(&FastSlamNode::publishMapToOdom, this)

            ); 
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        }


        private:

        int num_particles_;
     
        
        double a1_, a2_, a3_, a4_;
        
        double z_hit_ ;
        double std_hit_;
        double z_rand_ ;
        double p_rand_; // computed with scan range max
        
        // Scan update thresholds
        double linear_update_;
        double angular_update_;
        double resample_threshold_; 



        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; 
        std::unique_ptr<tf2_ros::TransformBroadcaster> map_odom_pub_; 
        geometry_msgs::msg::TransformStamped map_odom_tf_;
        rclcpp::TimerBase::SharedPtr tf_pub_timer_;
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


        // Motion
        Pose prev_pose_;
        Pose last_scan_pose_;
        bool initialized_ = false;
        bool scan_initialized_ = false;
        MotionModel md_;

        // Measurement
        ScanIntegrator integrator_; 
        std::random_device rd_;
        std::mt19937 gen_;

        std::vector<Particle> particles_; 
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
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

            if (delta_dist < linear_update_ && delta_rot < angular_update_) return;

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

        void scanCallback(const sensor_msgs::msg::LaserScan& scan) {
            if (!initialized_) return;

            // Only process scan if robot has moved enough
            double dx = prev_pose_.x - last_scan_pose_.x;
            double dy = prev_pose_.y - last_scan_pose_.y;
            double delta_dist = std::sqrt(dx*dx + dy*dy);
            double delta_rot = std::abs(prev_pose_.theta - last_scan_pose_.theta);
            if (scan_initialized_ && delta_dist < linear_update_) return;

            p_rand_ = 1.0/scan.range_max;
            geometry_msgs::msg::TransformStamped t;
            try {
                t = tf_buffer_->lookupTransform("base_footprint", "base_scan", tf2::TimePointZero, std::chrono::milliseconds(100));
                double dx = t.transform.translation.x;
                double dy = t.transform.translation.y;
                double d_theta = quatToYaw(
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w
                );

                double scan_x, scan_y, scan_theta;
                double scan_likelihood;
                double total_weight = 0.0;
                Particle* best_particle = nullptr;
                for (Particle& particle : particles_) {
                    scan_x = particle.x + dx * std::cos(particle.theta) - dy * std::sin(particle.theta);
                    scan_y = particle.y + dx * std::sin(particle.theta) + dy * std::cos(particle.theta);
                    scan_theta = particle.theta + d_theta;
                    scan_likelihood = computeLikelihood(scan, particle, scan_x, scan_y, scan_theta);
                    particle.w *= scan_likelihood;
                    total_weight += particle.w;
                    if (!best_particle || particle.w > best_particle->w) {
                        best_particle = &particle;
                    }
                }

                calculateOdomTf(best_particle);
                RCLCPP_INFO(this->get_logger(), "Best particle weight: %.6f", best_particle->w);
                RCLCPP_INFO(this->get_logger(), "Total weight in system: %.6f", total_weight);
                resample(total_weight);
                for (Particle& particle : particles_) {
                    // better performance to integrate scan after resampling || OR DOES IT...
                    integrator_.integrateScan(particle.map, scan, particle.x, particle.y, particle.theta);
                }
                publishMap(*best_particle);
                publishParticles();
                last_scan_pose_ = prev_pose_;
                scan_initialized_ = true;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform %s to %s: %s",
                    "base_footprint", "base_scan", ex.what()
                );
            }

        }

        double computeLikelihood(
            const sensor_msgs::msg::LaserScan& scan,
            Particle& particle,
            double scan_x, double scan_y, double scan_theta
        ) {
            double q = 1.0;
            double pz;
            double endpoint_x, endpoint_y;
            float dist; 
            for (unsigned int k = 0; k < scan.ranges.size(); k += 2) {
                pz = 0.0;
                if (scan.ranges[k] < scan.range_min || scan.ranges[k] > scan.range_max) continue;
                endpoint_x = scan_x + scan.ranges[k] * std::cos(scan_theta+ (scan.angle_min+scan.angle_increment*k));
                endpoint_y = scan_y + scan.ranges[k] * std::sin(scan_theta+ (scan.angle_min+scan.angle_increment*k));
                auto [endpoint_x_grid, endpoint_y_grid] = particle.map.worldToGridCoords(endpoint_x, endpoint_y);
                dist = particle.map.distanceAt(endpoint_x_grid, endpoint_y_grid);
                if (dist < 0) continue;
                pz += z_hit_ *  (std::exp(-(dist*dist)/ (2*(std_hit_*std_hit_))));
                pz += z_rand_*p_rand_; 
                assert(pz <= 1.0);
                assert(pz >= 0.0);
                // MAJOR CRE: Nav2 AMCL. this works better than logp with particle weights more spaced out.  
                q += pz*pz*pz; 
            }
            return q; 
        }


        void resample(double total_weight) {
            for (Particle& p : particles_) p.w /= total_weight;

            // Compute N_eff = 1 / sum(w_i^2)
            double sum_sq = 0.0;
            for (const Particle& p : particles_) sum_sq += p.w * p.w;
            double n_eff = 1.0 / sum_sq;
            RCLCPP_INFO(this->get_logger(), "N_eff: %.1f / %d", n_eff, num_particles_);

            if (n_eff >= resample_threshold_ * num_particles_) return;

            std::vector<double> cdf(num_particles_);
            std::vector<Particle> Xt;
            Xt.reserve(num_particles_);

            cdf[0] = particles_[0].w;
            for (int i = 1; i < num_particles_; i++) {
                cdf[i] = cdf[i-1] + particles_[i].w;
            }

            // Systematic resampler
            std::uniform_real_distribution<double> start_point_dist(0, 1.0/num_particles_);
            double start_point = start_point_dist(gen_);
            int curr_particle = 0;
            double u;
            for (int i = 0; i < num_particles_; i++) {
                u = start_point + i*(1.0/num_particles_);
                while (u > cdf[curr_particle] && curr_particle < num_particles_ - 1) curr_particle++;
                Xt.push_back(particles_[curr_particle]);
                Xt.back().w = 1.0/num_particles_;
            }
            particles_ = Xt;
            RCLCPP_INFO(this->get_logger(), "Resampled particles");
        }


        void calculateOdomTf(Particle* p) { 
            // map->odom = (map->p) * (odom->p)^-1
            geometry_msgs::msg::TransformStamped base_to_odom = tf_buffer_->lookupTransform(
                "base_footprint", // source   
                "odom", // target  
                tf2::TimePointZero,
                std::chrono::milliseconds(100)
            ); 
            // map->p
            double map_to_base_x = p->x;
            double map_to_base_y = p->y;
            double map_to_base_rot = p->theta;

            // (odom->p)^-1
            double base_to_odom_x = base_to_odom.transform.translation.x;
            double base_to_odom_y = base_to_odom.transform.translation.y; 
            double base_to_odom_rot = quatToYaw(
                base_to_odom.transform.rotation.x,
                base_to_odom.transform.rotation.y,
                base_to_odom.transform.rotation.z, 
                base_to_odom.transform.rotation.w
            );

            //map->odom 
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



        // ------------------ UTILS ------------------

        void publishMap(Particle& p) {
            auto map_msg = nav_msgs::msg::OccupancyGrid(); 
            auto header = std_msgs::msg::Header();
            auto map_meta_data = nav_msgs::msg::MapMetaData(); 
            header.stamp = this->now();
            header.frame_id = "map";
            fastslam::MapParams map_params = p.map.getMapParams(); 
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
            map_msg.data = p.map.toROSData();
            map_pub_->publish(map_msg); 
        }

        double getYaw(const nav_msgs::msg::Odometry& odom_msg) {
            double qx = odom_msg.pose.pose.orientation.x;
            double qy = odom_msg.pose.pose.orientation.y;
            double qz = odom_msg.pose.pose.orientation.z;
            double qw = odom_msg.pose.pose.orientation.w;
            return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        }

        
        double quatToYaw(double qx, double qy, double qz, double qw) {
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