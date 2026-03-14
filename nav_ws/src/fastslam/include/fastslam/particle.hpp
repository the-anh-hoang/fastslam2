#ifndef FASTSLAM_PARTICLE_HPP
#define FASTSLAM_PARTICLE_HPP
#include "fastslam/occupancy_grid_map.hpp"

namespace fastslam {

struct Particle {
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;
    double w     = 1.0;
    OccupancyGridMap map;

    explicit Particle(double x, double y, double theta, const MapParams& params = MapParams{}) : 
    x(x), y(y), theta(theta), map(params) {}
};

} // namespace fastslam
#endif

