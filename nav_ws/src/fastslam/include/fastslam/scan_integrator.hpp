#ifndef FASTSLAM_SCAN_INTEGRATOR_HPP
#define FASTSLAM_SCAN_INTEGRATOR_HPP
#include <fastslam/occupancy_grid_map.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace fastslam {

    class ScanIntegrator {
        public:
            ScanIntegrator(float l_occ, float l_free, int alpha,
                            double laser_dx, double laser_dy, double laser_dtheta);

            void integrateScan(
                OccupancyGridMap& occupancy_grid,
                const sensor_msgs::msg::LaserScan& scan,
                double robot_x,
                double robot_y,
                double robot_theta
            ); 
        
        private:
            float l_occ_;
            float l_free_;
            int alpha_;
            double laser_dx_;
            double laser_dy_;
            double laser_dtheta_;


            void rayCast(OccupancyGridMap& map,
                double x0, double y0,
                float ray_angle, 
                float range
            );  
    };

}

#endif 