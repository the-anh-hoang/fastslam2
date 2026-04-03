#ifndef FASTSLAM_MOTION_MODEL_HPP
#define FASTSLAM_MOTION_MODEL_HPP
#include <random>
#include "fastslam/pose.hpp"

namespace fastslam
{
    class MotionModel {
        public:
            MotionModel() {}
            explicit MotionModel(double a1, double a2, double a3, double a4) : 
                alpha_1_(a1), alpha_2_(a2), alpha_3_(a3), alpha_4_(a4) {}
            
            Pose applyMotionModel(Pose robot_pose, Pose prev_odom, Pose curr_odom); 

        private: 
            double alpha_1_, alpha_2_, alpha_3_, alpha_4_;
            std::mt19937 rng_;  
            
            double sampleNoise(double var);
    };
}

#endif