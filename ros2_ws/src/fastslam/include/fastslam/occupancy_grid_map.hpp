#ifndef FASTSLAM_OCCUPANCY_GRID_MAP_HPP
#define FASTSLAM_OCCUPANCY_GRID_MAP_HPP
#include <unordered_map>
#include <vector> 
#include <optional>
#include <cstdint>
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

    struct GridData {
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
            // it will be occupancygridmap job to find the chunk
            // and cell 
            void updateHit(double x_world, double y_world);
            
            // Get cell's center of mass
            std::optional<std::pair<double, double>> getMean(double x_world, double y_world) const;
            
            std::optional<std::pair<double, double>> kernelSearch(double x_world, double y_world, int kernel_size) const;

            // Convert world x, y to cell coordinates x, y
            std::pair<int, int> worldToGridCoords(double x, double y) const;
            
            // Return the exact grid coordinates in x, y
            std::pair<double, double> worldToGridCoordsExact(double x, double y) const;
            
            // Convert cell x,y to  world coordiantes x,y 
            std::pair<double, double> gridToWorldCoords(int x, int y) const;
            
            // Flatten the chunked log-odds map into a dense occupancy grid
            GridData toGridData() const;

            MapParams getMapParams() const;
            
            
        
        private:
            std::unordered_map<int64_t, OccupancyChunk> chunks_;
            MapParams map_params_;

            // kernelSearch memo: occupied-cell means per query cell, in kernel
            // iteration order. Valid only while the map is unmodified —
            // updateHit bumps map_version_, and the first query after that
            // clears the cache. One thread touches a map at a time.
            struct KernelCandidates { int n = 0; double pts[9][2]; };
            mutable std::unordered_map<int64_t, KernelCandidates> kernel_cache_;
            mutable uint64_t kernel_cache_version_ = 0;
            uint64_t map_version_ = 1;

            float max_dist_ = 0.0f; 
            bool distance_dirty_ = true;
            int grid_origin_x_, grid_origin_y_;  
            
            void getMapBoundingBox(int& min_x, int& min_y, int& max_x, int& max_y) const;

            static int64_t packKey(int cx, int cy);  
            int chunkIndex(int cell) const; 

            OccupancyChunk& getOrCreateChunk(int x, int y); 
            const OccupancyChunk* findChunk(int x, int y) const; 
            int localOffset(int cell) const; 
    };
}
#endif