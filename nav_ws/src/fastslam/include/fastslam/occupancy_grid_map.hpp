#ifndef FASTSLAM_OCCUPANCY_GRID_MAP_HPP
#define FASTSLAM_OCCUPANCY_GRID_MAP_HPP
#include <vector>
#include <cstdint>

namespace fastslam 
{
    struct MapParams {
        int   width      = 1000;
        int   height     = 1000;
        float resolution = 0.03f;
        float origin_x;
        float origin_y;
        float l_min      = -4.6f;
        float l_max      =  4.6f;

        MapParams() :   origin_x(-(width*resolution)/2.0f),
                        origin_y(-(height*resolution)/2.0f) {}
                        
        MapParams(int w, int h, float res) :
            width(w), height(h), resolution(res),    
            origin_x(-(width*resolution)/2.0f),
            origin_y(-(height*resolution)/2.0f) 
            {}
    };

    class OccupancyGridMap {
        public:
        
            explicit OccupancyGridMap(const MapParams& params);

            // Update cell's log odds value
            void accumulateLogOdds(int x, int y, float log_p);
            
            // Get cell's log odds value
            float getLogOdds(int x, int y) const;
            
            // Get distance at cell x, y
            float distanceAt(int x, int y);

            // Convert world x, y to cell coordinates x, y
            std::pair<int, int> worldToGridCoords(double x, double y) const;
            
            // Return the exact grid coordinates in x, y
            std::pair<double, double> worldToGridCoordsExact(double x, double y) const;
            
            // Convert cell x,y to  world coordiantes x,y 
            std::pair<double, double> gridToWorldCoords(double x, double y) const;
            
            // Convert data_ from log odds to ROS OccupancyGrid message data
            std::vector<int8_t> toROSData() const;

            MapParams getMapParams() const;
            
            // check if cell coords x, y are in bounds
            bool inBounds(int x, int y) const; 
        
        private:
            std::vector<float> data_;
            std::vector<float> distance_map_;
            MapParams map_params_;
            float max_dist_ = 0.0f; 
            bool distance_dirty_ = true;

            // Compute distance map for likelihood sensor model
            void computeDistanceMap();
    };
}
#endif