#ifndef FASTSLAM_SCAN_INTEGRATOR_HPP
#define FASTSLAM_SCAN_INTEGRATOR_HPP
#include <fastslam/occupancy_grid_map.hpp>
#include "fastslam/laser_scan.hpp"

namespace fastslam {

    class ScanIntegrator {
        public:
            ScanIntegrator() {}
            explicit ScanIntegrator(
                double laser_dx, double laser_dy, double laser_dtheta,
                int ray_skip
            ) :
                laser_dx_(laser_dx), laser_dy_(laser_dy), laser_dtheta_(laser_dtheta), 
                ray_skip_(ray_skip)
            {}

            
            void integrateScan(
                OccupancyGridMap& map,
                const LaserScan& scan,
                double robot_x,
                double robot_y,
                double robot_theta
            );
        private:
            double laser_dx_, laser_dy_, laser_dtheta_;
            int ray_skip_; 
    };

}

#endif 