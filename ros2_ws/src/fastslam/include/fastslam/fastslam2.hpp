#ifndef FASTSLAM_FASTSLAM2_HPP
#define FASTSLAM_FASTSLAM2_HPP
#include <cmath>
#include <random>
#include <string>
#include <vector>

#include "fastslam/laser_scan.hpp"
#include "fastslam/motion_model.hpp"
#include "fastslam/occupancy_grid_map.hpp"
#include "fastslam/particle.hpp"
#include "fastslam/pose.hpp"
#include "fastslam/scan_integrator.hpp"
#include "fastslam/scan_matcher.hpp"

namespace fastslam
{
    struct FastSlam2Config {
        int num_particles = 3;
        float map_chunk_size = 24.0f;   // m
        float map_res = 0.03f;          // m/cell

        // modes
        std::string mode = "proposal"; // proposal/peak
        std::string matcher = "correlative"; // correlative/gradient

        // RNG seed; < 0 means seed from std::random_device (non-reproducible)
        long seed = -1;

        // Weight tempering gains: weights are normalized with
        // exp((w - w_max) / (gain * N)) for N_eff and resampling respectively
        double neff_gain = 5.0;
        double resample_gain = 3.0;

        // Proposal-mode sampling window around the scan-match mode
        double proposal_range_xy = 0.1;
        double proposal_step_xy = 0.025;
        double proposal_range_theta = 0.05;
        double proposal_step_theta = 0.01;

        // Odometry motion model noise
        double a1 = 0.01, a2 = 0.01, a3 = 0.01, a4 = 0.01;

        // Laser mount offset in the robot base frame
        double laser_dx = 0.0, laser_dy = 0.0, laser_dtheta = 0.0;

        int ray_skip = 5;
        double scan_match_x_range = 0.09;
        double scan_match_y_range = 0.09;
        double scan_match_theta_range = M_PI/4;
        double scan_match_step_xy = 0.03;
        double scan_match_step_theta = 0.02;
        double z_hit = 0.8, std_hit = 0.2, z_rand = 0.2;

        // Motion gate: process a scan only after moving this far
        double linear_update = 0.5;
        double angular_update = 0.2;
        double resample_threshold = 0.5;
    };

    struct UpdateResult {
        enum class Status { Skipped, Initialized, Updated };
        Status status = Status::Skipped;

        // Diagnostics, only meaningful when status == Updated
        double n_eff = 0.0, n_eff_ratio = 0.0, spread = 0.0;
        double min_ll = 0.0, avg_ll = 0.0, max_ll = 0.0;      // scan-match log-likelihoods
        double avg_cov_xx = 0.0, avg_cov_yy = 0.0, avg_cov_tt = 0.0;
        double w_min = 0.0, w_max = 0.0;                      // normalized weight range
        bool did_resample = false;
        int scan_count = 0;
        int resample_count = 0;
        double total_distance = 0.0;
        Pose best_pose;
        double duration_ms = 0.0;
    };

    // FastSLAM 2.0 particle filter. Pure C++ — consumers feed it odometry
    // poses and laser scans and read back particles/maps.
    class FastSlam2 {
        public:
            explicit FastSlam2(const FastSlam2Config& config);

            // odom_pose: robot pose in the odom frame at the scan's time.
            // timestamp: scan time in seconds, used only for trajectory logging.
            UpdateResult processScan(const LaserScan& scan, const Pose& odom_pose, double timestamp);

            const Particle& bestParticle() const;
            const std::vector<Particle>& particles() const { return particles_; }

            // Write fastslam.tum (best particle trajectory) and odom.tum into dir
            void writeTrajectories(const std::string& dir) const;

        private:
            FastSlam2Config config_;

            MotionModel md_;
            ScanIntegrator integrator_;
            ScanMatcher scan_matcher_;

            std::mt19937 gen_;
            std::normal_distribution<double> std_normal_{0.0,1.0};

            std::vector<Particle> particles_;

            bool scan_initialized_ = false;
            Pose prev_pose_;

            // Trajectory logging — one entry per processed scan, so traj_stamps_[k]
            // pairs with Particle::poses[k+1] (poses[0] is the initial pose)
            std::vector<double> traj_stamps_;
            std::vector<Pose> odom_traj_;

            // Tracking
            int scan_count_ = 0;
            int resample_count_ = 0;
            double total_distance_ = 0.0;

            void resample();
            void writeTumFile(const std::string& path, const std::vector<Pose>& poses,
                              size_t pose_offset) const;
    };
}
#endif
