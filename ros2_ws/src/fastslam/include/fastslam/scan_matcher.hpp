#ifndef FASTSLAM_SCAN_MATCHER_HPP
#define FASTSLAM_SCAN_MATCHER_HPP
#include "fastslam/pose.hpp" 
#include "fastslam/particle.hpp"
#include "fastslam/occupancy_grid_map.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <array> 

namespace fastslam 
{
    // The Gaussian fit 
    struct ScanMatchResult { 
        // double total_region_w; 
        // Pose mean;  
        Pose best_pose;  
        // std::array<double, 9> cov;  
        double best_likelihood;

        ScanMatchResult() {}

        ScanMatchResult(Pose best, double best_likelihood)
        :   best_pose(best), best_likelihood(best_likelihood) {}
    };

    class ScanMatcher {
        public:
            
            ScanMatcher() {}

            ScanMatcher(
                int ray_skip, 
                double laser_dx, double laser_dy, double laser_dtheta, 
                double x_range, double y_range, double theta_range,
                double step_size_xy, double step_size_theta,
                double z_hit, double std_hit, double z_rand
            ) : ray_skip_(ray_skip), 
                laser_dx_(laser_dx), laser_dy_(laser_dy), laser_dtheta_(laser_dtheta), 
                x_range_(x_range), y_range_(y_range), theta_range_(theta_range),
                step_size_xy_(step_size_xy), step_size_theta_(step_size_theta),
                z_hit_(z_hit), std_hit_(std_hit), z_rand_(z_rand) 
            {}
            
            ScanMatchResult matchScan(
                Particle& particle, 
                const sensor_msgs::msg::LaserScan& scan
                
            );
            
            double computeLikelihood(
                OccupancyGridMap& map,
                double x, double y, double theta, 
                const sensor_msgs::msg::LaserScan& scan
            ); 

        private:


            int ray_skip_; 
            double laser_dx_, laser_dy_, laser_dtheta_; 
            double x_range_, y_range_, theta_range_; // search range
            double step_size_xy_, step_size_theta_; // search resolution 
            double z_hit_, std_hit_, z_rand_;
    };

}
#endif