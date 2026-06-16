#include "fastslam/scan_integrator.hpp"
#include "fastslam/angle_utils.hpp"
#include <cmath>
#include <limits>


namespace fastslam 
{

    void ScanIntegrator::integrateScan(
        OccupancyGridMap& map,
        const sensor_msgs::msg::LaserScan& scan,
        double robot_x, 
        double robot_y, 
        double robot_theta
    ) {
        double scan_x = robot_x + laser_dx_*std::cos(robot_theta) - laser_dy_*std::sin(robot_theta);
        double scan_y = robot_y + laser_dx_*std::sin(robot_theta) + laser_dy_*std::cos(robot_theta);
        double scan_theta = normalizeAngle(robot_theta + laser_dtheta_);
       
        double endpoint_x, endpoint_y;
        for (size_t i = 0; i < scan.ranges.size(); i+=ray_skip_) {
            if (scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max) continue;
            endpoint_x = scan_x + scan.ranges[i] * std::cos(scan_theta+ (scan.angle_min+scan.angle_increment*i));
            endpoint_y = scan_y + scan.ranges[i] * std::sin(scan_theta+ (scan.angle_min+scan.angle_increment*i));
            map.updateHit(endpoint_x, endpoint_y); 
        }

    }

  
}
