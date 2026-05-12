#include <vector>
#include <random>
#include <cmath>
#include <chrono>
#include <omp.h>
#include <fstream>
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
#include "fastslam/scan_matcher.hpp"
#include "fastslam/particle.hpp"
#include "fastslam/motion_model.hpp"
#include "fastslam/pose.hpp"



namespace fastslam {
    class FastSlam2Node : public rclcpp::Node {
        public:
        FastSlam2Node() : Node("fastslam2_node"), gen_(rd_()) {
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
            

            num_particles_ = this->get_parameter("num_particles").as_int();

            float map_chunk_size = static_cast<float>(this->get_parameter("map_chunk_size").as_double()); 
            float map_res = static_cast<float>(this->get_parameter("map_res").as_double()); 
            MapParams mp(map_chunk_size, map_res);
            md_ = MotionModel(
                this->get_parameter("a1").as_double(),
                this->get_parameter("a2").as_double(),            
                this->get_parameter("a3").as_double(),
                this->get_parameter("a4").as_double()
            );

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            double laser_dx, laser_dy, laser_dtheta; 
            try {
                geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                    "base_footprint",
                    "base_scan",
                    tf2::TimePointZero,
                    std::chrono::seconds(3)
                );

                laser_dx = t.transform.translation.x;
                laser_dy = t.transform.translation.y;
                laser_dtheta = quatToYaw(
                    t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w
                );
            } catch (const tf2::TransformException& ex) {
                RCLCPP_FATAL(this->get_logger(), "FAILED TO GET LASER TF: %s", ex.what());
                rclcpp::shutdown(); 
                return; 
            }
            RCLCPP_INFO(this->get_logger(), "Laser TF: dx=%.4f dy=%.4f dtheta=%.4f", 
                            laser_dx, laser_dy, laser_dtheta);
            scan_matcher_ = ScanMatcher(
                this->get_parameter("ray_skip").as_int(),
                laser_dx, laser_dy, laser_dtheta, 
                this->get_parameter("scan_match_x_range").as_double(),
                this->get_parameter("scan_match_y_range").as_double(),
                this->get_parameter("scan_match_theta_range").as_double(),
                this->get_parameter("scan_match_step_xy").as_double(),
                this->get_parameter("scan_match_step_theta").as_double(),
                this->get_parameter("z_hit").as_double(),
                this->get_parameter("std_hit").as_double(),
                this->get_parameter("z_rand").as_double()

            );
            integrator_ = ScanIntegrator(
                0.7f, -0.7f, 1,
                laser_dx, laser_dy, laser_dtheta,
                this->get_parameter("ray_skip").as_int()
            );
            
            linear_update_ = this->get_parameter("linear_update").as_double();
            angular_update_ = this->get_parameter("angular_update").as_double();

            resample_threshold_ = this->get_parameter("resample_threshold").as_double(); 
            RCLCPP_INFO(this->get_logger(), "num_particles: %d", num_particles_);
            RCLCPP_INFO(this->get_logger(), "linear_update: %.2f", linear_update_);
            
             
            particles_ = std::vector<Particle>(num_particles_, Particle(mp)); 
            csv_file_.open("diagnostics.csv");
            csv_file_ << "scan_id,particle_id,"
                    << "pre_motion_x,pre_motion_y,pre_motion_theta,"
                    << "odom_pred_x,odom_pred_y,odom_pred_theta,"
                    << "scan_match_x,scan_match_y,scan_match_theta,scan_match_ll,"
                    << "proposal_mean_x,proposal_mean_y,proposal_mean_theta,"
                    << "final_x,final_y,final_theta,"
                    << "log_eta,weight,"
                    << "cov_xx,cov_yy,cov_tt,"
                    << "odom_raw_x,odom_raw_y,odom_raw_theta" << std::endl;
            

            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(&FastSlam2Node::odomCallback, this, std::placeholders::_1)
            );
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


        private:

        std::ofstream csv_file_;
        std::vector<Pose> pre_motion_poses_;  // stored before motion model applies

        int num_particles_;
     
        
        // Scan update thresholds
        double linear_update_;
        double angular_update_;
        double resample_threshold_; 

        // Tracking
        int scan_count_ = 0;
        int resample_count_ = 0;
        double total_distance_ = 0.0;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; 
        std::unique_ptr<tf2_ros::TransformBroadcaster> map_odom_pub_; 
        geometry_msgs::msg::TransformStamped map_odom_tf_;
        rclcpp::TimerBase::SharedPtr tf_pub_timer_;
        
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


        // Motion
        bool odom_initialized_ = false;
        nav_msgs::msg::Odometry odom_msg_; 
        Pose prev_pose_;
        Pose last_scan_pose_;
        MotionModel md_;
        
        // Measurement
        bool scan_initialized_ = false;
        ScanIntegrator integrator_; 
        ScanMatcher scan_matcher_; 
        std::random_device rd_;
        std::mt19937 gen_;
        std::normal_distribution<double> std_normal_{0.0,1.0}; 

        std::vector<Particle> particles_; 
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;

        void odomCallback(const nav_msgs::msg::Odometry& odom_msg) {
            odom_msg_ = odom_msg; 
            if (!odom_initialized_) {
                odom_initialized_ = true;
                double curr_x = odom_msg.pose.pose.position.x;
                double curr_y = odom_msg.pose.pose.position.y;
                double curr_theta = getYaw(odom_msg); 
                prev_pose_ = Pose(curr_x, curr_y, curr_theta);
            }

            
            pre_motion_poses_.resize(num_particles_);
            for (int i = 0; i < num_particles_; i++) {
                pre_motion_poses_[i] = Pose(particles_[i].x, particles_[i].y, particles_[i].theta);
            }
 
        }

        void scanCallback(const sensor_msgs::msg::LaserScan& scan) {
            if (!odom_initialized_) return;

            // just integrate first scan 
            if (!scan_initialized_) {
                for (Particle& particle : particles_) {
                    integrator_.integrateScan(particle.map, scan, particle.x, particle.y, particle.theta);                 
                    scan_initialized_ = true;
                }
                return;
            }

            // Only process scan if robot has moved enough
            double curr_x = odom_msg_.pose.pose.position.x;
            double curr_y = odom_msg_.pose.pose.position.y;
            double curr_theta = getYaw(odom_msg_); 
            Pose curr_pose(curr_x, curr_y, curr_theta);

            double dx = curr_x - prev_pose_.x;
            double dy = curr_y - prev_pose_.y;
            double delta_dist = std::sqrt(dx*dx + dy*dy);
            double delta_rot = std::abs(prev_pose_.theta - curr_theta);
            if (delta_dist < linear_update_ && delta_rot < angular_update_) return;
            
            total_distance_ += delta_dist;
            scan_count_++;
            
            // sample motion model
            Pose sampled_pose;
            for (Particle& p : particles_) {
                sampled_pose = md_.applyMotionModel(
                    Pose(p.x, p.y, p.theta),
                    prev_pose_,
                    curr_pose
                );
                p.x = sampled_pose.x;
                p.y = sampled_pose.y;
                p.theta = sampled_pose.theta; 
            }
            prev_pose_ = curr_pose; 
            publishParticles(); 
            

            using Ms = std::chrono::duration<double, std::milli>;
            auto t_cb_start = std::chrono::steady_clock::now();

            // Pre-generate random normals — std::mt19937 is not thread-safe
            std::vector<double> randoms(num_particles_ * 3);
            for (int j = 0; j < num_particles_ * 3; j++) randoms[j] = std_normal_(gen_);

            // Per-particle diagnostics
            std::vector<double> particle_likelihoods(num_particles_);
            std::vector<double> particle_eta(num_particles_);
            std::vector<double> particle_cov_xx(num_particles_);
            std::vector<double> particle_cov_yy(num_particles_);
            std::vector<double> particle_cov_tt(num_particles_);
            std::vector<double> particle_scan_match_ll(num_particles_);

            double total_weight = 0.0;
            Particle* best_particle = nullptr;


            std::vector<double> csv_scan_match_x(num_particles_);
            std::vector<double> csv_scan_match_y(num_particles_);
            std::vector<double> csv_scan_match_theta(num_particles_);
            std::vector<double> csv_proposal_mean_x(num_particles_);
            std::vector<double> csv_proposal_mean_y(num_particles_);
            std::vector<double> csv_proposal_mean_theta(num_particles_);
            std::vector<double> csv_final_x(num_particles_);
            std::vector<double> csv_final_y(num_particles_);
            std::vector<double> csv_final_theta(num_particles_);
            std::vector<double> csv_odom_pred_x(num_particles_);
            std::vector<double> csv_odom_pred_y(num_particles_);
            std::vector<double> csv_odom_pred_theta(num_particles_);

            #pragma omp parallel for reduction(+:total_weight) schedule(static)
            for (int i = 0; i < num_particles_; i++) {
                Particle& particle = particles_[i];
                ScanMatchResult smr;
                double L[9];

                // scan matching
                smr = scan_matcher_.matchScan(particle, scan);
                double best_x = smr.best_pose.x;
                double best_y = smr.best_pose.y;
                double best_theta = smr.best_pose.theta;
                particle_scan_match_ll[i] = smr.best_likelihood;
                csv_scan_match_x[i] = best_x;
                csv_scan_match_y[i] = best_y;
                csv_scan_match_theta[i] = best_theta;
                csv_odom_pred_x[i] = particle.x;  // this IS the odom-predicted pose (motion model already applied)
                csv_odom_pred_y[i] = particle.y;
                csv_odom_pred_theta[i] = particle.theta;
                
                // -- SAMPLE AROUND THE MODE --
                std::vector<Pose> poses_sampled; poses_sampled.reserve(500);
                std::vector<double> log_weights; log_weights.reserve(500);
                double lmax = -std::numeric_limits<double>::infinity();
                double log_p_zt_xt, log_p_xt_ut;
                double dx, dy, dtheta, drho;
                for (double x = best_x-0.1; x < best_x+0.1; x+=0.025) {
                    for (double y = best_y-0.1; y < best_y+0.1; y+=0.025) {
                        for (double theta = best_theta-0.005; theta < best_theta+0.0051; theta+=0.0025) {
                            dx = x - particle.x;
                            dy = y - particle.y;
                            dtheta = std::atan2(std::sin(theta - particle.theta), std::cos(theta - particle.theta));
                            drho = dx*dx + dy*dy;
                            log_p_zt_xt = scan_matcher_.computeLikelihood(particle.map, x, y, theta, scan);
                            log_p_xt_ut = -0.1  * drho - 0.1 * dtheta * dtheta;
                            double lw = log_p_zt_xt + log_p_xt_ut;
                            poses_sampled.push_back(Pose(x,y,std::remainder(theta, 2.0*M_PI)));
                            log_weights.push_back(lw);
                            if (lw > lmax) lmax = lw;
                        }
                    }
                }

                // -- Computing the GAUSSIAN PROPOSAL -- 
                Pose mean_pose(0.0,0.0,0.0);
                double normalizing_term = 0.0; 
                std::vector<double> xj_probs; 
                xj_probs.reserve(poses_sampled.size()); 
                double total_sin = 0, total_cos = 0;
                for (size_t i = 0; i < poses_sampled.size(); i++) {
                    double p = std::exp(log_weights[i] - lmax); 
                    xj_probs.push_back(p);
                    mean_pose.x += p*poses_sampled[i].x;
                    mean_pose.y += p*poses_sampled[i].y;
                    total_sin += p*std::sin(poses_sampled[i].theta);
                    total_cos += p*std::cos(poses_sampled[i].theta);
                    normalizing_term += p;
                }

                mean_pose.x /= normalizing_term; mean_pose.y /= normalizing_term; 
                mean_pose.theta = std::atan2(total_sin/normalizing_term, total_cos/normalizing_term); 
                csv_proposal_mean_x[i] = mean_pose.x;
                csv_proposal_mean_y[i] = mean_pose.y;
                csv_proposal_mean_theta[i] = mean_pose.theta;
                // Covariance 
                std::array<double,9> cov = {0,0,0, 0,0,0, 0,0,0};
                
                for (size_t i = 0; i < poses_sampled.size(); i++) {
                    dx = poses_sampled[i].x - mean_pose.x;
                    dy = poses_sampled[i].y - mean_pose.y;
                    dtheta = std::atan2(
                        std::sin(poses_sampled[i].theta - mean_pose.theta),
                        std::cos(poses_sampled[i].theta - mean_pose.theta)
                    );
                    cov[0] += xj_probs[i] * dx     * dx; 
                    cov[1] += xj_probs[i] * dx     * dy; 
                    cov[2] += xj_probs[i] * dx     * dtheta; 
                    cov[3] += xj_probs[i] * dy     * dx; 
                    cov[4] += xj_probs[i] * dy     * dy; 
                    cov[5] += xj_probs[i] * dy     * dtheta; 
                    cov[6] += xj_probs[i] * dtheta * dx; 
                    cov[7] += xj_probs[i] * dtheta * dy; 
                    cov[8] += xj_probs[i] * dtheta * dtheta; 
                }
                
                for (int j = 0; j < 9; j++) cov[j] /= normalizing_term;
                cov[0] += 1e-6; cov[4] += 1e-6; cov[8] += 1e-6;
                
                // Store diagnostics
                particle_eta[i] = std::log(normalizing_term) + lmax;
                particle_cov_xx[i] = cov[0];
                particle_cov_yy[i] = cov[4];
                particle_cov_tt[i] = cov[8];
                
                // -- SAMPLING NEW POSE (CHOLESKY) -- 
                L[0] = std::sqrt(cov[0]);
                L[3] = cov[3] / L[0];
                L[4] = std::sqrt(cov[4] - L[3]*L[3]);
                L[6] = cov[6] / L[0];
                L[7] = (cov[7] - L[6]*L[3]) / L[4];
                L[8] = std::sqrt(cov[8] - L[6]*L[6] - L[7]*L[7]);
                double z0 = randoms[i*3];
                double z1 = randoms[i*3 + 1];
                double z2 = randoms[i*3 + 2];

                double sampled_x = mean_pose.x + L[0]*z0; 
                double sampled_y = mean_pose.y + L[3]*z0 + L[4]*z1;
                double sampled_theta = mean_pose.theta + L[6]*z0  + L[7]*z1 + L[8]*z2;
                
                particle.x = sampled_x; 
                particle.y = sampled_y;
                particle.theta = sampled_theta;

                csv_final_x[i] = sampled_x;
                csv_final_y[i] = sampled_y;
                csv_final_theta[i] = sampled_theta;
                particle.w += std::log(normalizing_term) + lmax;
                
                integrator_.integrateScan(particle.map, scan, particle.x, particle.y, particle.theta);
                total_weight += particle.w;
            }
            best_particle = &*std::max_element(particles_.begin(), particles_.end(),
                [](const Particle& a, const Particle& b) { return a.w < b.w; });
    
            calculateOdomTf(best_particle);

            // --- Compute N_eff before resampling ---
            double max_w = -std::numeric_limits<double>::infinity();
            for (const Particle& p : particles_) if (p.w > max_w) max_w = p.w;
            
            std::vector<double> normalized_w(num_particles_);
            double sum_w = 0.0;
            for (int i = 0; i < num_particles_; i++) {
                normalized_w[i] = std::exp(particles_[i].w - max_w);
                sum_w += normalized_w[i];
            }
            for (int i = 0; i < num_particles_; i++) normalized_w[i] /= sum_w;
            
            double sum_sq = 0.0;
            for (int i = 0; i < num_particles_; i++) sum_sq += normalized_w[i] * normalized_w[i];
            double n_eff = 1.0 / sum_sq;
            double n_eff_ratio = n_eff / num_particles_;

            // --- Particle spread (how far apart are the particles?) ---
            double mean_x = 0, mean_y = 0;
            for (const Particle& p : particles_) { mean_x += p.x; mean_y += p.y; }
            mean_x /= num_particles_; mean_y /= num_particles_;
            double spread = 0;
            for (const Particle& p : particles_) {
                double ddx = p.x - mean_x;
                double ddy = p.y - mean_y;
                spread += ddx*ddx + ddy*ddy;
            }
            spread = std::sqrt(spread / num_particles_);

            // --- Scan match quality stats ---
            double min_ll = *std::min_element(particle_scan_match_ll.begin(), particle_scan_match_ll.end());
            double max_ll = *std::max_element(particle_scan_match_ll.begin(), particle_scan_match_ll.end());
            double avg_ll = 0;
            for (double ll : particle_scan_match_ll) avg_ll += ll;
            avg_ll /= num_particles_;

            // --- Average proposal covariance (how tight is the proposal?) ---
            double avg_cov_xx = 0, avg_cov_yy = 0, avg_cov_tt = 0;
            for (int i = 0; i < num_particles_; i++) {
                avg_cov_xx += particle_cov_xx[i];
                avg_cov_yy += particle_cov_yy[i];
                avg_cov_tt += particle_cov_tt[i];
            }
            avg_cov_xx /= num_particles_;
            avg_cov_yy /= num_particles_;
            avg_cov_tt /= num_particles_;

            // --- Weight distribution ---
            double w_min = normalized_w[0], w_max = normalized_w[0];
            for (int i = 1; i < num_particles_; i++) {
                if (normalized_w[i] < w_min) w_min = normalized_w[i];
                if (normalized_w[i] > w_max) w_max = normalized_w[i];
            }

            // --- Resample ---
            bool did_resample = false;
            if (n_eff < resample_threshold_ * num_particles_) {
                resample();
                did_resample = true;
                resample_count_++;
            }

            publishMap(*best_particle);
            publishParticles();
            // last_scan_pose_ = prev_pose_;

            double t_total_ms = Ms(std::chrono::steady_clock::now() - t_cb_start).count();

            // === LOGGING ===
            RCLCPP_INFO(this->get_logger(),
                "[%d] %.1fms | dist=%.1fm | N_eff=%.1f/%d (%.0f%%) | spread=%.4fm | %s",
                scan_count_, t_total_ms, total_distance_,
                n_eff, num_particles_, n_eff_ratio * 100.0,
                spread,
                did_resample ? "RESAMPLED" : "");
            RCLCPP_INFO(this->get_logger(),
                "  scan_match: ll=[%.1f, %.1f, %.1f] | proposal_cov: σx=%.5f σy=%.5f σθ=%.5f",
                min_ll, avg_ll, max_ll,
                std::sqrt(avg_cov_xx), std::sqrt(avg_cov_yy), std::sqrt(avg_cov_tt));
            RCLCPP_INFO(this->get_logger(),
                "  weights: [%.4f, %.4f] | best=(%.2f,%.2f,%.2f) | resamples=%d/%d",
                w_min, w_max,
                best_particle->x, best_particle->y, best_particle->theta,
                resample_count_, scan_count_);
            if (csv_file_.is_open()) {
                for (int i = 0; i < num_particles_; i++) {
                    Pose pre = (pre_motion_poses_.size() == (size_t)num_particles_) 
                                ? pre_motion_poses_[i] 
                                : Pose(0,0,0);
                    csv_file_ << scan_count_ << "," << i << ","
                            << pre.x << "," << pre.y << "," << pre.theta << ","
                            << csv_odom_pred_x[i] << "," << csv_odom_pred_y[i] << "," << csv_odom_pred_theta[i] << ","
                            << csv_scan_match_x[i] << "," << csv_scan_match_y[i] << "," << csv_scan_match_theta[i] << "," << particle_scan_match_ll[i] << ","
                            << csv_proposal_mean_x[i] << "," << csv_proposal_mean_y[i] << "," << csv_proposal_mean_theta[i] << ","
                            << csv_final_x[i] << "," << csv_final_y[i] << "," << csv_final_theta[i] << ","
                            << particle_eta[i] << "," << particles_[i].w << ","
                            << particle_cov_xx[i] << "," << particle_cov_yy[i] << "," << particle_cov_tt[i] << ","
                            << prev_pose_.x << "," << prev_pose_.y << "," << prev_pose_.theta
                            << std::endl;
                    }
            }
        }


        void resample() {
            double max_w = -std::numeric_limits<double>::infinity();
            for (const Particle& p : particles_) if (p.w > max_w) max_w = p.w;

            std::vector<double> linear_w(num_particles_);
            double sum_w = 0.0;
            for (int i = 0; i < num_particles_; i++) {
                linear_w[i] = std::exp(particles_[i].w - max_w); 
                sum_w += linear_w[i];
            }

            for (int i = 0; i < num_particles_; i++) linear_w[i] /= sum_w;

            std::vector<double> cdf(num_particles_);
            cdf[0] = linear_w[0];
            for (int i = 1; i < num_particles_; i++) {
                cdf[i] = cdf[i-1] + linear_w[i];
            }

            // Systematic resampler
            std::vector<Particle> Xt;
            Xt.reserve(num_particles_); 
            std::uniform_real_distribution<double> start_point_dist(0, 1.0/num_particles_);
            double start_point = start_point_dist(gen_);
            int curr_particle = 0;
            double u;
            for (int i = 0; i < num_particles_; i++) {
                u = start_point + i*(1.0/num_particles_);
                while (u > cdf[curr_particle] && curr_particle < num_particles_ - 1) curr_particle++;
                Xt.push_back(particles_[curr_particle]);
                Xt.back().w = 0.0; 
            }
            particles_ = Xt;
        }


        void calculateOdomTf(Particle* p) { 
            geometry_msgs::msg::TransformStamped base_to_odom = tf_buffer_->lookupTransform(
                "base_footprint",
                "odom",
                tf2::TimePointZero,
                std::chrono::milliseconds(100)
            ); 
            double map_to_base_x = p->x;
            double map_to_base_y = p->y;
            double map_to_base_rot = p->theta;

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


        // ------------------ UTILS ------------------

        void publishMap(Particle& p) {
            auto map_msg = nav_msgs::msg::OccupancyGrid(); 
            auto header = std_msgs::msg::Header();
            auto map_meta_data = nav_msgs::msg::MapMetaData(); 
            header.stamp = this->now();
            header.frame_id = "map";
            MapParams map_params = p.map.getMapParams(); 
            ROSMsg map_ros_msg = p.map.toROSData();
            map_meta_data.map_load_time.sec = 0;
            map_meta_data.map_load_time.nanosec = 0;
            map_meta_data.resolution = map_params.resolution;
            map_meta_data.width = map_ros_msg.width;
            map_meta_data.height = map_ros_msg.height;
            map_meta_data.origin.position.x = map_ros_msg.origin_x;
            map_meta_data.origin.position.y = map_ros_msg.origin_y;
            map_meta_data.origin.position.z = 0.0;
            map_meta_data.origin.orientation.x = 0; 
            map_meta_data.origin.orientation.y = 0; 
            map_meta_data.origin.orientation.z = 0; 
            map_meta_data.origin.orientation.w = 1;

            map_msg.header = header; 
            map_msg.info = map_meta_data;
            map_msg.data = map_ros_msg.data;
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
    rclcpp::spin(std::make_shared<fastslam::FastSlam2Node>());
    rclcpp::shutdown();
    return 0;
}