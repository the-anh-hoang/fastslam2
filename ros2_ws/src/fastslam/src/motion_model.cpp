#include "fastslam/motion_model.hpp"
#include <cmath>
#include <algorithm> // needed for std::min

static double normalizeAngle(double angle) {
    return std::remainder(angle, 2.0 * M_PI);
}

namespace fastslam 
{
    Pose MotionModel::applyMotionModel(Pose robot_pose, Pose prev_odom, Pose curr_odom) {
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

        // Apply Noise
        double noisy_rot1  = delta_rot1 + sampleNoise(
            alpha_1_*delta_rot1_noise*delta_rot1_noise + alpha_2_*delta_trans*delta_trans
        );
        double noisy_trans = delta_trans + sampleNoise(
            alpha_3_*delta_trans*delta_trans + alpha_4_*(delta_rot1_noise*delta_rot1_noise + delta_rot2_noise*delta_rot2_noise)
        );
        double noisy_rot2  = delta_rot2  + sampleNoise(
            alpha_1_*delta_rot2_noise*delta_rot2_noise + alpha_2_*delta_trans*delta_trans
        );

        double new_x     = robot_pose.x + noisy_trans * std::cos(robot_pose.theta + noisy_rot1);
        double new_y     = robot_pose.y + noisy_trans * std::sin(robot_pose.theta + noisy_rot1);
        double new_theta = normalizeAngle(robot_pose.theta + noisy_rot1 + noisy_rot2);
        
        return Pose(new_x, new_y, new_theta); 
    } 

    double MotionModel::sampleNoise(double var) {
        std::normal_distribution<double> dist(0.0, std::sqrt(var));
        return dist(rng_); 
    }
}