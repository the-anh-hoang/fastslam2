#ifndef FASTSLAM_ANGLE_UTILS_HPP
#define FASTSLAM_ANGLE_UTILS_HPP
#include <cmath>

namespace fastslam
{
    // Wrap an angle to (-pi, pi].
    inline double normalizeAngle(double angle) {
        return std::remainder(angle, 2.0 * M_PI);
    }
}

#endif
