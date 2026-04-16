#include "fastslam/scan_matcher.hpp" 
#include <cmath> 
#include <cassert> 

namespace fastslam 
{

    ScanMatchResult ScanMatcher::matchScan(
        Particle& particle, 
        const sensor_msgs::msg::LaserScan& scan
    ) {

        
        double start_x = particle.x - x_range_;
        double end_x = particle.x + x_range_;
        
        double start_y = particle.y - y_range_;
        double end_y = particle.y + y_range_;

        double start_theta = particle.theta - theta_range_;
        double end_theta = particle.theta + theta_range_; 

        Pose best_pose(0.0, 0.0, 0.0);
        double best_likelihood = -std::numeric_limits<double>::infinity(); 
        double log_p;
        for (double x = start_x; x < end_x; x+=step_size_xy_) {
            for (double y = start_y; y < end_y; y+=step_size_xy_) {
                for (double theta = start_theta; theta < end_theta; theta+=step_size_theta_) {
                   
                    log_p = computeLikelihood(particle.map, x, y, theta, scan); 
                    if (log_p > best_likelihood) {
                        best_likelihood = log_p;
                        best_pose.x = x; 
                        best_pose.y = y;
                        best_pose.theta = theta; 
                    }  
                }
            }
        }
        return ScanMatchResult(best_pose, best_likelihood); 
    }

    
    double ScanMatcher::computeLikelihood(
            OccupancyGridMap& map,
            double x, double y, double theta, 
            const sensor_msgs::msg::LaserScan& scan
    ) {
        double p_rand = 1.0/scan.range_max; 
        double scan_x = x + laser_dx_ * std::cos(theta) - laser_dy_ * std::sin(theta);
        double scan_y = y + laser_dx_ * std::sin(theta) + laser_dy_ * std::cos(theta);
        double scan_theta = theta + laser_dtheta_;
        double q = 0.0;
        double pz;
        double endpoint_x, endpoint_y;
        float dist; 
        for (unsigned int k = 0; k < scan.ranges.size(); k += ray_skip_) {
            pz = 0.0;
            if (scan.ranges[k] < scan.range_min || scan.ranges[k] > scan.range_max) continue;
            endpoint_x = scan_x + scan.ranges[k] * std::cos(scan_theta+ (scan.angle_min+scan.angle_increment*k));
            endpoint_y = scan_y + scan.ranges[k] * std::sin(scan_theta+ (scan.angle_min+scan.angle_increment*k));
            auto [endpoint_x_grid, endpoint_y_grid] = map.worldToGridCoords(endpoint_x, endpoint_y);
            dist = map.distanceAt(endpoint_x_grid, endpoint_y_grid);
            if (dist < 0) continue;
            pz += z_hit_ *  (std::exp(-(dist*dist)/ (2*(std_hit_*std_hit_))));
            pz += z_rand_*p_rand; 
            assert(pz <= 1.0);
            assert(pz >= 0.0); 
            q += std::log(pz); 
        }
        return q; 
    }
}