#include "fastslam/motion_model.hpp"
#include <cmath>
#include <algorithm> 

static double normalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

namespace fastslam 
{
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
        double hat_dx = odom_pred.x - prev_pose.x;
        double hat_dy = odom_pred.y - prev_pose.y;
        double hat_trans = std::sqrt(hat_dx*hat_dx + hat_dy*hat_dy);
        double hat_rot1 = (hat_trans > 0.001) 
            ? normalizeAngle(std::atan2(hat_dy, hat_dx) - prev_pose.theta) 
            : 0.0;
        double hat_rot2 = normalizeAngle(odom_pred.theta - prev_pose.theta - hat_rot1);

        double cand_dx = candidate_pose.x - prev_pose.x;
        double cand_dy = candidate_pose.y - prev_pose.y;
        double cand_trans = std::sqrt(cand_dx*cand_dx + cand_dy*cand_dy);
        double cand_rot1 = (cand_trans > 0.001) 
            ? normalizeAngle(std::atan2(cand_dy, cand_dx) - prev_pose.theta) 
            : 0.0;
        double cand_rot2 = normalizeAngle(candidate_pose.theta - prev_pose.theta - cand_rot1);

        double var_rot1  = alpha_1_ * hat_rot1*hat_rot1   + alpha_2_ * hat_trans*hat_trans;
        double var_trans = alpha_3_ * hat_trans*hat_trans  + alpha_4_ * (hat_rot1*hat_rot1 + hat_rot2*hat_rot2);
        double var_rot2  = alpha_1_ * hat_rot2*hat_rot2   + alpha_2_ * hat_trans*hat_trans;

        double diff_rot1  = normalizeAngle(cand_rot1 - hat_rot1);
        double diff_trans = cand_trans - hat_trans;
        double diff_rot2  = normalizeAngle(cand_rot2 - hat_rot2);

        double log_p =  -0.5 * diff_rot1*diff_rot1   / std::max(var_rot1,  1e-6)
                      + -0.5 * diff_trans*diff_trans  / std::max(var_trans, 1e-6)
                      + -0.5 * diff_rot2*diff_rot2    / std::max(var_rot2,  1e-6);

        return log_p;
    }

    double MotionModel::sampleNoise(double var) {
        std::normal_distribution<double> dist(0.0, std::sqrt(var));
        return dist(rng_); 
    }
}