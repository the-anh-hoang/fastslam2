#include "fastslam/fastslam2.hpp"
#include "fastslam/angle_utils.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace fastslam
{

    FastSlam2::FastSlam2(const FastSlam2Config& config)
        : config_(config),
          gen_(config.seed >= 0 ? static_cast<std::mt19937::result_type>(config.seed)
                                : std::mt19937::result_type{std::random_device{}()})
    {
        MapParams mp(config_.map_chunk_size, config_.map_res);
        md_ = MotionModel(config_.a1, config_.a2, config_.a3, config_.a4);
        scan_matcher_ = ScanMatcher(
            config_.ray_skip,
            config_.laser_dx, config_.laser_dy, config_.laser_dtheta,
            config_.scan_match_x_range,
            config_.scan_match_y_range,
            config_.scan_match_theta_range,
            config_.scan_match_step_xy,
            config_.scan_match_step_theta,
            config_.z_hit, config_.std_hit, config_.z_rand
        );
        integrator_ = ScanIntegrator(
            config_.laser_dx, config_.laser_dy, config_.laser_dtheta,
            config_.ray_skip
        );
        particles_ = std::vector<Particle>(config_.num_particles, Particle(0.0,0.0,0.0,mp));
    }

    const Particle& FastSlam2::bestParticle() const {
        return *std::max_element(particles_.begin(), particles_.end(),
            [](const Particle& a, const Particle& b) { return a.w < b.w; });
    }

    UpdateResult FastSlam2::processScan(const LaserScan& scan, const Pose& odom_pose, double timestamp) {
        const int num_particles = config_.num_particles;
        UpdateResult res;
        Pose curr_pose = odom_pose;

        // just integrate first scan
        if (!scan_initialized_) {
            // prev_pose_ tracking odom history, not particle poses history
            prev_pose_ = curr_pose;
            Pose particle_pose;
            for (Particle& particle : particles_) {
                particle_pose = particle.poses.back();
                // 3 times to trigger threshold as anchor
                integrator_.integrateScan(particle.map, scan, particle_pose.x, particle_pose.y, particle_pose.theta);
                integrator_.integrateScan(particle.map, scan, particle_pose.x, particle_pose.y, particle_pose.theta);
                integrator_.integrateScan(particle.map, scan, particle_pose.x, particle_pose.y, particle_pose.theta);

                scan_initialized_ = true;
            }
            res.status = UpdateResult::Status::Initialized;
            res.best_pose = particles_[0].poses.back();
            return res;
        }
        // Only process scan if robot has moved enough
        double curr_x = curr_pose.x;
        double curr_y = curr_pose.y;
        double curr_theta = curr_pose.theta;

        double dx = curr_x - prev_pose_.x;
        double dy = curr_y - prev_pose_.y;
        double delta_dist = std::sqrt(dx*dx + dy*dy);
        double delta_rot = std::abs(normalizeAngle(curr_theta - prev_pose_.theta));
        if (delta_dist < config_.linear_update && std::abs(delta_rot) < config_.angular_update) {
            res.status = UpdateResult::Status::Skipped;
            return res;
        }

        total_distance_ += delta_dist;
        scan_count_++;
        traj_stamps_.push_back(timestamp);
        odom_traj_.push_back(curr_pose);


        using Ms = std::chrono::duration<double, std::milli>;
        auto t_start = std::chrono::steady_clock::now();

        // Pre-generate random normals — std::mt19937 is not thread-safe
        std::vector<double> randoms(num_particles * 3);
        for (int j = 0; j < num_particles * 3; j++) randoms[j] = std_normal_(gen_);

        // Per-particle diagnostics
        std::vector<double> particle_eta(num_particles);
        std::vector<double> particle_cov_xx(num_particles);
        std::vector<double> particle_cov_yy(num_particles);
        std::vector<double> particle_cov_tt(num_particles);
        std::vector<double> particle_scan_match_ll(num_particles);

        double total_weight = 0.0;
        for (Particle& p : particles_) {
            bool deterministic = false; 
            if (config_.mode == "proposal") {deterministic = true;}
            Pose odom_predicted_pose = md_.applyMotionModel(
                p.poses.back(),
                prev_pose_,
                curr_pose,
                deterministic
            );
            
            p.poses.push_back(
                Pose(
                    odom_predicted_pose.x,
                    odom_predicted_pose.y,
                    odom_predicted_pose.theta
                )
            );
        }
        #pragma omp parallel for reduction(+:total_weight) schedule(static)
        for (int i = 0; i < num_particles; i++) {
            Particle& particle = particles_[i];


            ScanMatchResult smr;
            double L[9];

            // scan matching
            if (config_.matcher == "gradient") {
                smr = scan_matcher_.matchScanGradient(particle, scan);
            } else if (config_.matcher == "grid") {
                smr = scan_matcher_.matchScanGrid(particle, scan);
            } else {
                smr = scan_matcher_.matchScanCorrelative(particle, scan);
            }
            double best_x = smr.best_pose.x;
            double best_y = smr.best_pose.y;
            double best_theta = smr.best_pose.theta;
            particle_scan_match_ll[i] = smr.best_likelihood;
            // TODO: MISSING CASE FOR FAILED SCAN MATCHER
            
            if (config_.mode == "proposal") {
                std::vector<Pose> poses_sampled; poses_sampled.reserve(500);
                std::vector<double> log_weights; log_weights.reserve(500);
                double lmax = -std::numeric_limits<double>::infinity();
                double lw, log_p_zt_xt, log_p_xt_ut;
                double dx, dy, dtheta;
                Pose particle_pose = particle.poses.back();
                const double pr_xy = config_.proposal_range_xy, ps_xy = config_.proposal_step_xy;
                const double pr_th = config_.proposal_range_theta, ps_th = config_.proposal_step_theta;
                for (double x = best_x-pr_xy; x < best_x+pr_xy; x+=ps_xy) {
                    for (double y = best_y-pr_xy; y < best_y+pr_xy; y+=ps_xy) {
                        for (double theta = best_theta-pr_th; theta < best_theta+pr_th; theta+=ps_th) {
                            dx = x - particle_pose.x;
                            dy = y - particle_pose.y;
                            dtheta = std::atan2(std::sin(theta - particle_pose.theta), std::cos(theta - particle_pose.theta));

                            log_p_zt_xt = scan_matcher_.computeLikelihood(particle.map, x, y, theta, scan)/3;
                            log_p_xt_ut = md_.evaluateLogMotionError(
                                particle.poses[particle.poses.size() - 2],
                                Pose(x, y, theta),
                                particle_pose
                            ); // how likely is z,y,theta based on particle movement from poses[-2] to poses[-1] (curr)

                            lw = (log_p_zt_xt + log_p_xt_ut);
                            poses_sampled.push_back(Pose(x,y,normalizeAngle(theta)));
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
                for (size_t j = 0; j < poses_sampled.size(); j++) {
                    double p = std::exp(log_weights[j] - lmax);
                    xj_probs.push_back(p);
                    mean_pose.x += p*poses_sampled[j].x;
                    mean_pose.y += p*poses_sampled[j].y;
                    total_sin += p*std::sin(poses_sampled[j].theta);
                    total_cos += p*std::cos(poses_sampled[j].theta);
                    normalizing_term += p;
                }

                mean_pose.x /= normalizing_term; mean_pose.y /= normalizing_term;
                mean_pose.theta = std::atan2(total_sin/normalizing_term, total_cos/normalizing_term);

                // Covariance
                std::array<double,9> cov = {
                    0,0,0,
                    0,0,0,
                    0,0,0
                };

                for (size_t j = 0; j < poses_sampled.size(); j++) {
                    dx = poses_sampled[j].x - mean_pose.x;
                    dy = poses_sampled[j].y - mean_pose.y;
                    dtheta = std::atan2(
                        std::sin(poses_sampled[j].theta - mean_pose.theta),
                        std::cos(poses_sampled[j].theta - mean_pose.theta)
                    );
                    cov[0] += xj_probs[j] * dx     * dx;
                    cov[1] += xj_probs[j] * dx     * dy;
                    cov[2] += xj_probs[j] * dx     * dtheta;
                    cov[3] += xj_probs[j] * dy     * dx;
                    cov[4] += xj_probs[j] * dy     * dy;
                    cov[5] += xj_probs[j] * dy     * dtheta;
                    cov[6] += xj_probs[j] * dtheta * dx;
                    cov[7] += xj_probs[j] * dtheta * dy;
                    cov[8] += xj_probs[j] * dtheta * dtheta;
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

                particle.poses.back() = Pose(
                    sampled_x,
                    sampled_y,
                    normalizeAngle(sampled_theta)
                );
                particle.w += (std::log(normalizing_term) + lmax);
            } else if (config_.mode == "peak"){
                particle.poses.back() = Pose(
                    best_x,
                    best_y,
                    best_theta
                );
                particle.w += smr.best_likelihood; 
            } else {
                throw std::invalid_argument("Unknown mode: " + config_.mode);
            }
            Pose particle_pose = particle.poses.back(); 
            integrator_.integrateScan(particle.map, scan, particle_pose.x, particle_pose.y, particle_pose.theta);
            total_weight += particle.w;            
        }
        prev_pose_ = curr_pose;

        // --- Compute N_eff before resampling ---
        double max_w = -std::numeric_limits<double>::infinity();
        for (const Particle& p : particles_) if (p.w > max_w) max_w = p.w;

        std::vector<double> normalized_w(num_particles);
        double sum_w = 0.0;
        for (int i = 0; i < num_particles; i++) {
            normalized_w[i] = std::exp((particles_[i].w - max_w)/ (config_.neff_gain*num_particles));
            sum_w += normalized_w[i];
        }
        for (int i = 0; i < num_particles; i++) normalized_w[i] /= sum_w;

        double sum_sq = 0.0;
        for (int i = 0; i < num_particles; i++) sum_sq += normalized_w[i] * normalized_w[i];
        double n_eff = 1.0 / sum_sq;

        // --- Particle spread (how far apart are the particles?) ---
        double mean_x = 0, mean_y = 0;
        for (const Particle& p : particles_) {
            Pose particle_pose = p.poses.back();
            mean_x += particle_pose.x; mean_y += particle_pose.y;
        }
        mean_x /= num_particles; mean_y /= num_particles;
        double spread = 0;
        for (const Particle& p : particles_) {
            Pose particle_pose = p.poses.back();
            double ddx = particle_pose.x - mean_x;
            double ddy = particle_pose.y - mean_y;
            spread += ddx*ddx + ddy*ddy;
        }
        spread = std::sqrt(spread / num_particles);

        // --- Scan match quality stats ---
        double min_ll = *std::min_element(particle_scan_match_ll.begin(), particle_scan_match_ll.end());
        double max_ll = *std::max_element(particle_scan_match_ll.begin(), particle_scan_match_ll.end());
        double avg_ll = 0;
        for (double ll : particle_scan_match_ll) avg_ll += ll;
        avg_ll /= num_particles;

        // --- Average proposal covariance (how tight is the proposal?) ---
        double avg_cov_xx = 0, avg_cov_yy = 0, avg_cov_tt = 0;
        for (int i = 0; i < num_particles; i++) {
            avg_cov_xx += particle_cov_xx[i];
            avg_cov_yy += particle_cov_yy[i];
            avg_cov_tt += particle_cov_tt[i];
        }
        avg_cov_xx /= num_particles;
        avg_cov_yy /= num_particles;
        avg_cov_tt /= num_particles;

        // --- Weight distribution ---
        double w_min = normalized_w[0], w_max = normalized_w[0];
        for (int i = 1; i < num_particles; i++) {
            if (normalized_w[i] < w_min) w_min = normalized_w[i];
            if (normalized_w[i] > w_max) w_max = normalized_w[i];
        }

        res.best_pose = bestParticle().poses.back();

        // --- Resample ---
        bool did_resample = false;
        if (n_eff < config_.resample_threshold * num_particles) {
            resample();
            did_resample = true;
            resample_count_++;
        }

        res.status = UpdateResult::Status::Updated;
        res.n_eff = n_eff;
        res.n_eff_ratio = n_eff / num_particles;
        res.spread = spread;
        res.min_ll = min_ll;
        res.avg_ll = avg_ll;
        res.max_ll = max_ll;
        res.avg_cov_xx = avg_cov_xx;
        res.avg_cov_yy = avg_cov_yy;
        res.avg_cov_tt = avg_cov_tt;
        res.w_min = w_min;
        res.w_max = w_max;
        res.did_resample = did_resample;
        res.scan_count = scan_count_;
        res.resample_count = resample_count_;
        res.total_distance = total_distance_;
        res.duration_ms = Ms(std::chrono::steady_clock::now() - t_start).count();
        return res;
    }


    void FastSlam2::resample() {
        const int num_particles = config_.num_particles;
        double max_w = -std::numeric_limits<double>::infinity();
        for (const Particle& p : particles_) if (p.w > max_w) max_w = p.w;

        std::vector<double> linear_w(num_particles);
        double sum_w = 0.0;
        for (int i = 0; i < num_particles; i++) {
            linear_w[i] = std::exp((particles_[i].w - max_w) / (config_.resample_gain*num_particles));
            sum_w += linear_w[i];
        }

        for (int i = 0; i < num_particles; i++) linear_w[i] /= sum_w;

        std::vector<double> cdf(num_particles);
        cdf[0] = linear_w[0];
        for (int i = 1; i < num_particles; i++) {
            cdf[i] = cdf[i-1] + linear_w[i];
        }

        // Systematic resampler
        std::vector<Particle> Xt;
        Xt.reserve(num_particles);
        std::uniform_real_distribution<double> start_point_dist(0, 1.0/num_particles);
        double start_point = start_point_dist(gen_);
        int curr_particle = 0;
        double u;
        for (int i = 0; i < num_particles; i++) {
            u = start_point + i*(1.0/num_particles);
            while (u > cdf[curr_particle] && curr_particle < num_particles - 1) curr_particle++;
            Xt.push_back(particles_[curr_particle]);
            Xt.back().w = 0.0;
        }
        particles_ = Xt;
    }


    void FastSlam2::writeTumFile(const std::string& path, const std::vector<Pose>& poses,
                                 size_t pose_offset) const {
        size_t n = std::min(traj_stamps_.size(), poses.size() - pose_offset);
        std::ofstream f(path);
        if (!f) return;
        f << std::fixed << std::setprecision(6);
        for (size_t k = 0; k < n; k++) {
            const Pose& p = poses[k + pose_offset];
            f << traj_stamps_[k] << " " << p.x << " " << p.y << " 0 0 0 "
              << std::sin(p.theta / 2.0) << " " << std::cos(p.theta / 2.0) << "\n";
        }
    }

    void FastSlam2::writeTrajectories(const std::string& dir) const {
        if (traj_stamps_.empty() || dir.empty()) return;
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        writeTumFile(dir + "/fastslam.tum", bestParticle().poses, 1);
        writeTumFile(dir + "/odom.tum", odom_traj_, 0);
    }

}
