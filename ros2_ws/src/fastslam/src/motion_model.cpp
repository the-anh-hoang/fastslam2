#include "fastslam/motion_model.hpp"
#include "fastslam/angle_utils.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
namespace fastslam
{
    static double foldRotation(double rot) {
        return std::min(std::abs(normalizeAngle(rot)),
                        std::abs(normalizeAngle(rot - M_PI)));
    }
    Pose MotionModel::applyMotionModel(Pose robot_pose, Pose prev_odom, Pose curr_odom, bool deterministic) {
        if (deterministic) {
            double delta_x = curr_odom.x - prev_odom.x;
            double delta_y = curr_odom.y - prev_odom.y;
            double delta_theta = normalizeAngle(curr_odom.theta - prev_odom.theta);

            // Raw Kinematics
            double delta_trans = std::sqrt(delta_x*delta_x + delta_y*delta_y);
            // in place rotation, only account rot2 noise
            double delta_rot1;
            if (delta_trans < 0.02)
                delta_rot1 = 0.0;
            else
                delta_rot1 = normalizeAngle(std::atan2(delta_y, delta_x) - prev_odom.theta);
            double delta_rot2 = normalizeAngle(delta_theta - delta_rot1);

            double new_x     = robot_pose.x + delta_trans * std::cos(robot_pose.theta + delta_rot1);
            double new_y     = robot_pose.y + delta_trans * std::sin(robot_pose.theta + delta_rot1);
            double new_theta = normalizeAngle(robot_pose.theta + delta_rot1 + delta_rot2);
            return Pose(new_x, new_y, new_theta);
        }

        // GMapping's MotionModel::drawFromMotion(p, pnew, pold): perturb the odom
        // delta per-axis in prev_odom's frame, then compose onto the particle pose.
        // Coefficients are stddev-style multipliers, not Thrun variances —
        // roles: srr trans->trans (a3), str rot->trans (a4),
        //        srt trans->rot   (a2), stt rot->rot   (a1)
        double srr = alpha_3_, str = alpha_4_, srt = alpha_2_, stt = alpha_1_;
        double sxy = 0.3 * srr;

        // absoluteDifference(curr_odom, prev_odom)
        double s = std::sin(prev_odom.theta), c = std::cos(prev_odom.theta);
        double raw_dx = curr_odom.x - prev_odom.x;
        double raw_dy = curr_odom.y - prev_odom.y;
        double dx     =  c*raw_dx + s*raw_dy;
        double dy     = -s*raw_dx + c*raw_dy;
        double dtheta = normalizeAngle(curr_odom.theta - prev_odom.theta);

        // sigmas come from the clean delta; cross-terms couple the axes
        double noisy_dx = dx + sampleGaussian(
            srr*std::abs(dx) + str*std::abs(dtheta) + sxy*std::abs(dy));
        double noisy_dy = dy + sampleGaussian(
            srr*std::abs(dy) + str*std::abs(dtheta) + sxy*std::abs(dx));
        double noisy_dtheta = dtheta + sampleGaussian(
            stt*std::abs(dtheta) + srt*std::sqrt(dx*dx + dy*dy));
        noisy_dtheta = std::fmod(noisy_dtheta, 2.0*M_PI);
        if (noisy_dtheta > M_PI) noisy_dtheta -= 2.0*M_PI;

        // absoluteSum(robot_pose, noisy delta)
        double ps = std::sin(robot_pose.theta), pc = std::cos(robot_pose.theta);
        return Pose(
            robot_pose.x + pc*noisy_dx - ps*noisy_dy,
            robot_pose.y + ps*noisy_dx + pc*noisy_dy,
            normalizeAngle(robot_pose.theta + noisy_dtheta)
        );
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

        double lp_a = logGaussian(cand_rot1, cand_trans, cand_rot2);
        double lp_b = logGaussian(normalizeAngle(cand_rot1 + M_PI),
                                -cand_trans,
                                normalizeAngle(cand_rot2 - M_PI));
        return std::max(lp_a, lp_b);
    }

    double MotionModel::sampleGaussian(double sigma) {
        if (sigma <= 0.0) return 0.0;
        // thread_local: applyMotionModel runs inside the node's OpenMP loop
        thread_local std::mt19937 rng(std::random_device{}());
        std::normal_distribution<double> dist(0.0, sigma);
        return dist(rng);
    }
}