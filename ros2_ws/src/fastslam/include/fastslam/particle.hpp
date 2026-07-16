#ifndef FASTSLAM_PARTICLE_HPP
#define FASTSLAM_PARTICLE_HPP
#include "fastslam/occupancy_grid_map.hpp"
#include "fastslam/pose.hpp"

namespace fastslam {

struct Particle {
    double w     = 0.0;
    OccupancyGridMap map;
    std::vector<Pose> poses;  
    Particle(MapParams params) : map(params) {}
    explicit Particle(double x, double y, double theta, const MapParams& params = MapParams{}) : 
    map(params) {poses.push_back(Pose(x,y,theta));}
};

} // namespace fastslam
#endif

