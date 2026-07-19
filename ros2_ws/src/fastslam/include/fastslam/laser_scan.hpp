#ifndef FASTSLAM_LASER_SCAN_HPP
#define FASTSLAM_LASER_SCAN_HPP
#include <vector>

namespace fastslam
{
    // mirrors the fields of sensor_msgs/LaserScan the
    // fastslam core actually uses. The ROS node converts at the boundary.
    struct LaserScan {
        double range_min = 0.0;
        double range_max = 0.0;
        double angle_min = 0.0;
        double angle_increment = 0.0;
        std::vector<float> ranges;
    };
}
#endif
