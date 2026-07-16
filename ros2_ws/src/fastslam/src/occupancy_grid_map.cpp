#include "fastslam/occupancy_grid_map.hpp"
#include <limits>
#include <cmath>

namespace fastslam 
{

    OccupancyGridMap::OccupancyGridMap(const MapParams& params) {
        map_params_ = params;
        chunks_[packKey(0,0)] = OccupancyChunk(map_params_.cells_per_side); 
    }

    void OccupancyGridMap::updateHit(double x_world, double y_world) {
        auto [x_grid, y_grid] = worldToGridCoords(x_world, y_world); 
        OccupancyChunk& chunk = getOrCreateChunk(x_grid, y_grid);
        int lx_grid = localOffset(x_grid);
        int ly_grid = localOffset(y_grid);

        chunk.updateHit(x_world, y_world, lx_grid, ly_grid);  
    }

    // get the mean of the cell that x and y fall into
    std::optional<std::pair<double,double>> OccupancyGridMap::getMean(double x_world, double y_world) const {
        auto [x_grid, y_grid] = worldToGridCoords(x_world, y_world); 
        const OccupancyChunk* chunk = findChunk(x_grid,y_grid);
        if (!chunk) return std::nullopt;
        return chunk->getMean(localOffset(x_grid), localOffset(y_grid));
    }

    std::optional<std::pair<double, double>> OccupancyGridMap::kernelSearch(double x_world, double y_world, int kernel_size) const {
        auto [x_grid, y_grid] = worldToGridCoords(x_world, y_world);
        const OccupancyChunk* chunk;
        double closest_dist = std::numeric_limits<double>::infinity(); 
        std::optional<std::pair<double, double>> closest_point;
        std::optional<std::pair<double, double>> candidate_point; 
        for (int dx = -kernel_size; dx <= kernel_size; dx++) {
            for (int dy = -kernel_size; dy <= kernel_size; dy++) {
                chunk = findChunk(x_grid+dx, y_grid+dy);
                if (!chunk) continue;
                candidate_point = chunk->getMean(localOffset(x_grid+dx), localOffset(y_grid+dy));
                if (!candidate_point) continue; 
                auto [x,y] = *candidate_point;
                double dist = sqrt((x-x_world)*(x-x_world) + (y-y_world)*(y-y_world));
                if (dist < closest_dist) {
                    closest_dist = dist;
                    closest_point = {x,y}; 
                }
            }
        }
        return closest_point; 
    }
        
        
    

    std::pair<int, int> OccupancyGridMap::worldToGridCoords(double x, double y) const {
        return {
            static_cast<int>(std::floor((x - map_params_.origin_x) / map_params_.resolution)),
            static_cast<int>(std::floor((y - map_params_.origin_y) / map_params_.resolution))
        };
    }

    std::pair<double, double> OccupancyGridMap::worldToGridCoordsExact(double x, double y) const {
        return {
            (x-map_params_.origin_x)/map_params_.resolution,
            (y-map_params_.origin_y)/map_params_.resolution 
        };
    }

    std::pair<double, double> OccupancyGridMap::gridToWorldCoords(int x, int y) const {
        return {
            map_params_.origin_x + x*map_params_.resolution,
            map_params_.origin_y + y*map_params_.resolution
        };
    }

    GridData OccupancyGridMap::toGridData() const {
        GridData grid;
        int min_x, min_y, max_x, max_y;
        getMapBoundingBox(min_x, min_y, max_x, max_y);
        int width = max_x - min_x;
        int height = max_y - min_y;
        std::pair<double,double> world_origin = gridToWorldCoords(min_x, min_y);
        grid.origin_x = world_origin.first;
        grid.origin_y = world_origin.second;
        grid.width = max_x - min_x;
        grid.height = max_y - min_y;
        grid.data.resize(width*height,-1);


        for (int x = min_x; x < max_x; x++) {
            for (int y = min_y; y < max_y; y++) {
                const OccupancyChunk* chunk = findChunk(x, y);
                if (!chunk) continue;
                if (chunk->isOccupied(localOffset(x), localOffset(y), 1)) {
                    grid.data[(y - min_y) * width + (x - min_x)] = 100;
                }
            }
        }
        return grid;
    }

    MapParams OccupancyGridMap::getMapParams() const {
        return map_params_;
    }

    void OccupancyGridMap::getMapBoundingBox(int& min_x, int& min_y, int& max_x, int& max_y) const {
        if (chunks_.empty()) {
            min_x = min_y = max_x = max_y = 0;
            return;
        } 
        int min_cx = std::numeric_limits<int>::max();
        int min_cy = std::numeric_limits<int>::max();

        int max_cx = std::numeric_limits<int>::lowest();
        int max_cy = std::numeric_limits<int>::lowest();

        // find the min/max chunk index
        for (const auto& [key, _] : chunks_) {
            int cx = (int) (key >> 32);
            int cy = (int) (key & 0xFFFFFFFF);
            min_cx = std::min(min_cx, cx);
            min_cy = std::min(min_cy, cy);
            max_cx = std::max(max_cx, cx);
            max_cy = std::max(max_cy, cy);
        }

        // convert them into world cell coords 
        min_x = min_cx * map_params_.cells_per_side;
        min_y = min_cy * map_params_.cells_per_side;
        max_x = (max_cx + 1) * map_params_.cells_per_side;
        max_y = (max_cy + 1) * map_params_.cells_per_side;
    }

    int64_t OccupancyGridMap::packKey(int cx, int cy) { 
        return ((int64_t) cx << 32) | (uint32_t) cy; 
    }


    OccupancyChunk& OccupancyGridMap::getOrCreateChunk(int x, int y) {
        int cx = chunkIndex(x); 
        int cy = chunkIndex(y); 
        int64_t key = packKey(cx, cy);
        auto it = chunks_.find(key);
        if (it == chunks_.end()) {
            chunks_[key] = OccupancyChunk(map_params_.cells_per_side); 
        }
        return chunks_[key]; 
    }

    const OccupancyChunk* OccupancyGridMap::findChunk(int x, int y) const {
        int64_t key = packKey(chunkIndex(x), chunkIndex(y));
        auto it = chunks_.find(key);
        if (it == chunks_.end()) return nullptr;
        return &it->second;
    }

    int OccupancyGridMap::chunkIndex(int cell) const {
        if (cell < 0) {
            return (cell - map_params_.cells_per_side + 1)/ map_params_.cells_per_side;
        } 
        return cell / map_params_.cells_per_side;
    }

    int OccupancyGridMap::localOffset(int cell) const {
        int res = cell % map_params_.cells_per_side;
        if (res < 0) res += map_params_.cells_per_side;
        return res; 
    }
}
