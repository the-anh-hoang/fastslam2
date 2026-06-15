#ifndef OCCUPANCY_CHUNK_HPP
#define OCCUPANCY_CHUNK_HPP 

#include <vector> 
#include <optional>
#include "fastslam/cell.hpp"

namespace fastslam 
{
    // size x size chunk of center of mass cells
    struct OccupancyChunk {
        int size; 
        std::vector<Cell> cells;

        OccupancyChunk() : size(0) {} 

        explicit OccupancyChunk(int cells_per_side) 
            : size(cells_per_side), cells(cells_per_side * cells_per_side) {}        

        std::optional<std::pair<double, double>> getMean(int local_x, int local_y) const {
            const Cell& c = cells[local_y * size + local_x];
            if (c.isOccupied(3)) {
                return cells[local_y * size + local_x].getMean(); 
            } else {
                return std::nullopt; 
            }
        }

        void updateHit(double x, double y, int local_x, int local_y) {
            cells[local_y * size + local_x].updateHit(x,y); 
        }
    };

} 

 #endif 