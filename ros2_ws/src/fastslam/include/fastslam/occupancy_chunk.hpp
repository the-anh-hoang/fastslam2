#ifndef OCCUPANCY_CHUNK_HPP
#define OCCUPANCY_CHUNK_HPP 

#include <vector> 

namespace fastslam 
{
    // LOG ODDS OCCUPANCY CHUNK 
    struct OccupancyChunk {
        int size; // cells per side
        std::vector<float> cells;

        OccupancyChunk() : size(0) {} 

        explicit OccupancyChunk(int cells_per_side) 
            : size(cells_per_side), cells(cells_per_side * cells_per_side, 0.0f) {}        

        float get(int local_x, int local_y) const {
            return cells[local_y * size + local_x]; 
        }

        void set(int local_x, int local_y, float value) {
            cells[local_y * size + local_x] = value; 
        }
    };

} 

 #endif 