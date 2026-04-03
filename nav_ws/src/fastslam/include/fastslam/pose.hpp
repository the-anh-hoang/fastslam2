#ifndef FASTSLAM_POSE_HPP
#define FASTSLAM_POSE_HPP

namespace fastslam
{
    struct Pose {
        double x, y, theta;
        Pose() {}
        Pose(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    };
}

#endif
