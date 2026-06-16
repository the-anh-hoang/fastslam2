#include "fastslam/scan_matcher.hpp" 
#include "fastslam/angle_utils.hpp"
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


    ScanMatchResult ScanMatcher::matchScanGradient(
        Particle& particle,
        const sensor_msgs::msg::LaserScan& scan
    ) {
        Pose current_pose(particle.x, particle.y, particle.theta);
        double current_score = computeLikelihood(
            particle.map, current_pose.x, current_pose.y, current_pose.theta, scan
        );

        // Initial step = search reach; refined by halving down to the
        // step_size_* precision floors. The climb is not bounded by the
        // initial step: it keeps walking while the likelihood improves.
        double ldelta = std::max(x_range_, y_range_);
        double adelta = theta_range_;

        while (ldelta >= step_size_xy_ || adelta >= step_size_theta_) {
            bool improved = true;
            while (improved) {
                improved = false;
                const double moves[6][3] = {
                    { ldelta,     0.0,     0.0},
                    {-ldelta,     0.0,     0.0},
                    {    0.0,  ldelta,     0.0},
                    {    0.0, -ldelta,     0.0},
                    {    0.0,     0.0,  adelta},
                    {    0.0,     0.0, -adelta}
                };
                // Greedy: accept improvements mid-sweep, so later probes
                // start from the updated pose (same as GMapping's optimize).
                for (const double* m : moves) {
                    double x = current_pose.x + m[0];
                    double y = current_pose.y + m[1];
                    double theta = current_pose.theta + m[2];
                    double score = computeLikelihood(particle.map, x, y, theta, scan);
                    if (score > current_score) {
                        current_score = score;
                        current_pose = Pose(x, y, theta);
                        improved = true;
                    }
                }
            }
            ldelta *= 0.5;
            adelta *= 0.5;
        }

        return ScanMatchResult(current_pose, current_score);
    }


    double ScanMatcher::computeLikelihood(
            OccupancyGridMap& map,
            double x, double y, double theta, 
            const sensor_msgs::msg::LaserScan& scan
    ) {
        double p_rand = 1.0/scan.range_max; 
        double scan_x = x + laser_dx_ * std::cos(theta) - laser_dy_ * std::sin(theta);
        double scan_y = y + laser_dx_ * std::sin(theta) + laser_dy_ * std::cos(theta);
        double scan_theta = normalizeAngle(theta + laser_dtheta_);
        double q = 0.0;
        double pz;
        double endpoint_x, endpoint_y;
        float dist; 


        std::optional<std::pair<double, double>> closest_point; 

        for (unsigned int k = 0; k < scan.ranges.size(); k += ray_skip_) {
            if (scan.ranges[k] < scan.range_min || scan.ranges[k] > scan.range_max) continue;
            
            endpoint_x = scan_x + scan.ranges[k] * std::cos(scan_theta+ (scan.angle_min+scan.angle_increment*k));
            endpoint_y = scan_y + scan.ranges[k] * std::sin(scan_theta+ (scan.angle_min+scan.angle_increment*k));

            closest_point = map.kernelSearch(endpoint_x, endpoint_y, 1);
            if (!closest_point) continue;
            auto [x_closest,y_closest] = *closest_point;
            dist = sqrt((x_closest-endpoint_x)*(x_closest-endpoint_x) + (y_closest-endpoint_y)*(y_closest-endpoint_y)); 
            
            pz = z_hit_ *  (std::exp(-(dist*dist)/ (2*(std_hit_*std_hit_))));
            pz += z_rand_*p_rand; 
            assert(pz <= 1.0);
            assert(pz >= 0.0); 
            q += std::log(pz) - std::log(z_rand_*p_rand); 

        }
        return q;
    }
}