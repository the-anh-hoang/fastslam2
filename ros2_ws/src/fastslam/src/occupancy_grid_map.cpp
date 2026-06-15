#include "fastslam/occupancy_grid_map.hpp"
#include <opencv2/opencv.hpp>

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

    ROSMsg OccupancyGridMap::toROSData() const {
        ROSMsg ros_msg; 
        // int min_x, min_y, max_x, max_y;
        // getMapBoundingBox(min_x, min_y, max_x, max_y);
        // int width = max_x - min_x;
        // int height = max_y - min_y;
        // std::pair<double,double> world_origin = gridToWorldCoords(min_x, min_y); 
        // ros_msg.origin_x =  world_origin.first;
        // ros_msg.origin_y = world_origin.second; 
        // ros_msg.width = max_x - min_x;
        // ros_msg.height = max_y - min_y; 
        // ros_msg.data.resize(width*height,-1);
        
        // float p; 
        // int idx;
        // for (int x = min_x; x < max_x; x++) {
        //     for (int y = min_y; y < max_y; y++) {
        //         idx = (y - min_y) * width + (x - min_x);
        //         p = 1.0f/(1.0f + std::exp(-getLogOdds(x,y)));
        //         if (p > 0.7) {
        //             ros_msg.data[idx] = 100;
        //         } else if (p < 0.3) {
        //             ros_msg.data[idx] = 0;
        //         } else {
        //             ros_msg.data[idx] = -1; 
        //         }
        //     }
        // }
        return ros_msg; 
    }

    MapParams OccupancyGridMap::getMapParams() const {
        return map_params_;
    }

    void OccupancyGridMap::getMapBoundingBox(int& min_x, int& min_y, int& max_x, int& max_y) const {
        if (chunks_.empty()) {
            min_x = min_y = max_x = max_y = 0;
            return;
        } 
        int min_cx = INT_MAX, min_cy = INT_MAX;
        int max_cx = INT_MIN, max_cy = INT_MIN;

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
