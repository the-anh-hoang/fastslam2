#include "fastslam/motion_model.hpp"
#include <cmath>

static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
}

namespace fastslam 
{
    Pose MotionModel::applyMotionModel(Pose robot_pose, Pose prev_pose, Pose curr_pose) {
            double delta_x = curr_pose.x - prev_pose.x;
            double delta_y = curr_pose.y - prev_pose.y;
            double delta_theta = curr_pose.theta - prev_pose.theta; 

            double delta_trans = std::sqrt(delta_x*delta_x + delta_y*delta_y);
            double delta_rot1 = normalizeAngle(
                (delta_trans > 0.01) ? std::atan2(delta_y, delta_x) - prev_pose.theta : 0.0
            );
            double delta_rot2 = normalizeAngle(delta_theta - delta_rot1);

            
            
            double noisy_rot1  = delta_rot1 + sampleNoise(alpha_1_*delta_rot1*delta_rot1 
                                                        + alpha_2_*delta_trans*delta_trans);

            double noisy_trans = delta_trans + sampleNoise(alpha_3_*delta_trans*delta_trans 
                                                        + alpha_4_*delta_rot1*delta_rot1 
                                                        + alpha_4_*delta_rot2*delta_rot2);

            double noisy_rot2  = delta_rot2  + sampleNoise(alpha_1_*delta_rot2*delta_rot2 
                                              + alpha_2_*delta_trans*delta_trans);

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