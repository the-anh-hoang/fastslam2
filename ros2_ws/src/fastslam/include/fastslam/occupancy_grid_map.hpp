#ifndef FASTSLAM_OCCUPANCY_GRID_MAP_HPP
#define FASTSLAM_OCCUPANCY_GRID_MAP_HPP
#include <unordered_map>
#include <vector> 
#include <cstdint>
#include <opencv2/opencv.hpp> 
#include "fastslam/occupancy_chunk.hpp"

namespace fastslam 
{
    struct MapParams {
        float chunk_size_m = 24.0f; // m 
        float resolution = 0.03f;
        int cells_per_side = static_cast<int>(chunk_size_m / resolution); 
        float origin_x;
        float origin_y;
        float l_min      = -4.6f;
        float l_max      =  4.6f;

        MapParams() :   origin_x(-chunk_size_m/2.0f),
                        origin_y(-chunk_size_m/2.0f) {}
                        
        MapParams(float chunk_size, float res) :
            chunk_size_m(chunk_size), 
            resolution(res),    
            origin_x(-chunk_size_m/2.0f),
            origin_y(-chunk_size_m/2.0f) 
            {}
    };

    struct ROSMsg {
        std::vector<int8_t> data; 
        float origin_x; 
        float origin_y; 
        int width;
        int height;
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
            std::pair<double, double> gridToWorldCoords(int x, int y) const;
            
            // Convert data_ from log odds to ROS OccupancyGrid message data
            ROSMsg toROSData() const;

            MapParams getMapParams() const;
            
            
        
        private:
            std::unordered_map<int64_t, OccupancyChunk> chunks_; 
            MapParams map_params_;

            cv::Mat dist_mat_;
            float max_dist_ = 0.0f; 
            bool distance_dirty_ = true;
            int grid_origin_x_, grid_origin_y_;  
            
            void getMapBoundingBox(int& min_x, int& min_y, int& max_x, int& max_y) const;

            static int64_t packKey(int cx, int cy);  
            int chunkIndex(int cell) const; 
            // Compute distance map for likelihood sensor model


            OccupancyChunk& getOrCreateChunk(int x, int y); 
            const OccupancyChunk* findChunk(int x, int y) const; 
            int localOffset(int cell) const; 
            void computeDistanceMap();
    };
}
#endif