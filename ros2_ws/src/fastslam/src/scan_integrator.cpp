#include "fastslam/scan_integrator.hpp"
#include <cmath>
#include <limits>

static int getSign(float x) {
    return (x==0) ? 0 : static_cast<int>(x/std::abs(x)); 
}
namespace fastslam 
{

    void ScanIntegrator::integrateScan(
        OccupancyGridMap& occupancy_grid,
        const sensor_msgs::msg::LaserScan& scan,
        double robot_x, 
        double robot_y, 
        double robot_theta
    ) {
        double scan_x = robot_x + laser_dx_*std::cos(robot_theta) - laser_dy_*std::sin(robot_theta);
        double scan_y = robot_y + laser_dx_*std::sin(robot_theta) + laser_dy_*std::cos(robot_theta);
        double scan_theta = robot_theta + laser_dtheta_;

        auto [scan_x_grid, scan_y_grid] = occupancy_grid.worldToGridCoordsExact(scan_x, scan_y);
        float ray_ang;
        for (int i = 0; i < scan.ranges.size(); i+=ray_skip_) {
            if (scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max) continue;
            ray_ang = static_cast<float>(scan_theta + (scan.angle_min + scan.angle_increment*i));
            while (ray_ang > M_PI) ray_ang -= 2*M_PI;
            while (ray_ang < -M_PI) ray_ang += 2*M_PI;
            float range_cells = scan.ranges[i] / occupancy_grid.getMapParams().resolution; 
            rayCast(occupancy_grid, scan_x_grid, scan_y_grid, ray_ang, range_cells);
        }

    }

    void ScanIntegrator::rayCast(OccupancyGridMap& map,
        double x0, double y0,
        float ray_angle, 
        float range
    ) {
        float rdx = std::cos(ray_angle);
        float rdy = std::sin(ray_angle);
        double r0_fract_x = x0-std::floor(x0); 
        double r0_fract_y = y0-std::floor(y0);
        float step_unit_x = (rdx == 0) ? 
            std::numeric_limits<float>::infinity() : std::sqrt(1 + (rdy/rdx)*(rdy/rdx));
        float step_unit_y = (rdy == 0) ? 
            std::numeric_limits<float>::infinity() : std::sqrt(1 + (rdx/rdy)*(rdx/rdy));

        int curr_x = static_cast<int>(std::floor(x0));
        int curr_y = static_cast<int>(std::floor(y0)); 
        float dist_x = (rdx > 0) ? (1-r0_fract_x)*step_unit_x : r0_fract_x*step_unit_x;
        float dist_y = (rdy > 0) ? (1-r0_fract_y)*step_unit_y : r0_fract_y*step_unit_y;
        float dist_traversed = 0;
        while (dist_traversed < range) {
            map.accumulateLogOdds(curr_x, curr_y, l_free_); 
            if (dist_y < dist_x) {
                curr_y += getSign(rdy);
                dist_traversed = dist_y;
                dist_y += step_unit_y;
            } else {
                curr_x += getSign(rdx);
                dist_traversed = dist_x;
                dist_x += step_unit_x;
            }
        }

        
        for (int dy = -alpha_; dy <= alpha_; dy++) {
            for (int dx = -alpha_; dx <= alpha_; dx++) {
                map.accumulateLogOdds(curr_x+dx, curr_y+dy, l_occ_); 
            }
        }
        
    }
}
