#include "fastslam/motion_model.hpp"
#include "fastslam/angle_utils.hpp"
#include <cmath>
#include <algorithm>

namespace fastslam
{
    static double foldRotation(double rot) {
        return std::min(std::abs(normalizeAngle(rot)),
                        std::abs(normalizeAngle(rot - M_PI)));
    }
    Pose MotionModel::applyMotionModel(Pose robot_pose, Pose prev_odom, Pose curr_odom, bool deterministic) {
        double delta_x = curr_odom.x - prev_odom.x;
        double delta_y = curr_odom.y - prev_odom.y;
        double delta_theta = normalizeAngle(curr_odom.theta - prev_odom.theta); 
        
        // Raw Kinematics
        double delta_trans = std::sqrt(delta_x*delta_x + delta_y*delta_y);
        double delta_rot1 = normalizeAngle(
            (delta_trans > 0.001) ? std::atan2(delta_y, delta_x) - prev_odom.theta : 0.0
        );
        double delta_rot2 = normalizeAngle(delta_theta - delta_rot1);

        // Noise penalty 
        // if not moving backwards would explode delta rotations
        // CREDIT: Nav2's motion model
        double delta_rot1_noise = std::min(
            std::abs(normalizeAngle(delta_rot1 - 0.0)),
            std::abs(normalizeAngle(delta_rot1 - M_PI))
        );
        double delta_rot2_noise = std::min(
            std::abs(normalizeAngle(delta_rot2 - 0.0)),
            std::abs(normalizeAngle(delta_rot2 - M_PI))
        );

        double noisy_rot1; 
        double noisy_trans;
        double noisy_rot2; 
        if (deterministic) {
            noisy_rot1 = delta_rot1;
            noisy_trans = delta_trans;
            noisy_rot2 = delta_rot2;
        } else {
            // Apply Noise
            noisy_rot1  = delta_rot1 + sampleNoise(
                alpha_1_*delta_rot1_noise*delta_rot1_noise + alpha_2_*delta_trans*delta_trans
            );
            noisy_trans = delta_trans + sampleNoise(
                alpha_3_*delta_trans*delta_trans + alpha_4_*(delta_rot1_noise*delta_rot1_noise + delta_rot2_noise*delta_rot2_noise)
            );
            noisy_rot2  = delta_rot2  + sampleNoise(
                alpha_1_*delta_rot2_noise*delta_rot2_noise + alpha_2_*delta_trans*delta_trans
            );
        }

        double new_x     = robot_pose.x + noisy_trans * std::cos(robot_pose.theta + noisy_rot1);
        double new_y     = robot_pose.y + noisy_trans * std::sin(robot_pose.theta + noisy_rot1);
        double new_theta = normalizeAngle(robot_pose.theta + noisy_rot1 + noisy_rot2);
        
        return Pose(new_x, new_y, new_theta); 
    } 

    

    double MotionModel::evaluateLogMotionError(Pose prev_pose, Pose candidate_pose, Pose odom_pred) {
        // Odometry-predicted decomposition = the sampler's mean
        double hat_dx = odom_pred.x - prev_pose.x;
        double hat_dy = odom_pred.y - prev_pose.y;
        double hat_trans = std::sqrt(hat_dx*hat_dx + hat_dy*hat_dy);
        double hat_rot1 = (hat_trans > 0.001)
            ? normalizeAngle(std::atan2(hat_dy, hat_dx) - prev_pose.theta)
            : 0.0;
        double hat_rot2 = normalizeAngle(odom_pred.theta - prev_pose.theta - hat_rot1);

        // Variances from FOLDED rotations — must match applyMotionModel exactly
        double r1n = foldRotation(hat_rot1);
        double r2n = foldRotation(hat_rot2);
        double var_rot1  = alpha_1_ * r1n*r1n + alpha_2_ * hat_trans*hat_trans;
        double var_trans = alpha_3_ * hat_trans*hat_trans + alpha_4_ * (r1n*r1n + r2n*r2n);
        double var_rot2  = alpha_1_ * r2n*r2n + alpha_2_ * hat_trans*hat_trans;

        // Candidate decomposition (note: always yields cand_trans >= 0)
        double cand_dx = candidate_pose.x - prev_pose.x;
        double cand_dy = candidate_pose.y - prev_pose.y;
        double cand_trans = std::sqrt(cand_dx*cand_dx + cand_dy*cand_dy);
        double cand_rot1 = (cand_trans > 0.001)
            ? normalizeAngle(std::atan2(cand_dy, cand_dx) - prev_pose.theta)
            : 0.0;
        double cand_rot2 = normalizeAngle(candidate_pose.theta - prev_pose.theta - cand_rot1);

        auto logGaussian = [&](double r1, double t, double r2) {
            double d1 = normalizeAngle(r1 - hat_rot1);
            double dt = t - hat_trans;
            double d2 = normalizeAngle(r2 - hat_rot2);
            return - 0.5 * d1*d1 / std::max(var_rot1,  1e-6)
                - 0.5 * dt*dt / std::max(var_trans, 1e-6)
                - 0.5 * d2*d2 / std::max(var_rot2,  1e-6);
        };

        // (rot1, trans, rot2) and (rot1+π, -trans, rot2-π) produce the identical
        // pose. The sqrt/atan2 decomposition above always picks the +trans branch,
        // so a candidate sitting *behind* prev_pose decomposes with rot1 ≈ ±π and
        // would be annihilated against a forward hat (diff ≈ π). Evaluate both
        // branches and keep the dominant one.
        double lp_a = logGaussian(cand_rot1, cand_trans, cand_rot2);
        double lp_b = logGaussian(normalizeAngle(cand_rot1 + M_PI),
                                -cand_trans,
                                normalizeAngle(cand_rot2 - M_PI));
        return std::max(lp_a, lp_b);
    }

    double MotionModel::sampleNoise(double var) {
        std::normal_distribution<double> dist(0.0, std::sqrt(var));
        return dist(rng_); 
    }
}