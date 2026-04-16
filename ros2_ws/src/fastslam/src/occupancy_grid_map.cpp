#include "fastslam/occupancy_grid_map.hpp"
#include <opencv2/opencv.hpp>

namespace fastslam 
{

    OccupancyGridMap::OccupancyGridMap(const MapParams& params) {
        map_params_ = params;
        data_.resize(map_params_.width * map_params_.height, 0.0f);
        distance_map_.resize(map_params_.width * map_params_.height, 0.0f); 
    }

    void OccupancyGridMap::accumulateLogOdds(int x, int y, float log_p) {
        data_[y*map_params_.width + x] = std::clamp(
            data_[y*map_params_.width + x] + log_p,
            map_params_.l_min, map_params_.l_max
        ); 
        distance_dirty_ = true; 
    }

    float OccupancyGridMap::getLogOdds(int x, int y) const {
        return data_[y*map_params_.width + x]; 
    }

    float OccupancyGridMap::distanceAt(int x, int y) {
        if (!inBounds(x, y)) return max_dist_;
        if (distance_dirty_) {
            computeDistanceMap();
        } 
        return distance_map_[y*map_params_.width + x]; 
    }

    std::pair<int, int> OccupancyGridMap::worldToGridCoords(double x, double y) const {
        return {
            static_cast<int>((x-map_params_.origin_x)/map_params_.resolution),
            static_cast<int>((y-map_params_.origin_y)/map_params_.resolution)
        };
    }

    std::pair<double, double> OccupancyGridMap::worldToGridCoordsExact(double x, double y) const {
        return {
        (x-map_params_.origin_x)/map_params_.resolution,
        (y-map_params_.origin_y)/map_params_.resolution 
        };
    }

    std::pair<double, double> OccupancyGridMap::gridToWorldCoords(double x, double y) const {
        return {
            map_params_.origin_x + x*map_params_.resolution,
            map_params_.origin_y + y*map_params_.resolution
        };
    }

    std::vector<int8_t> OccupancyGridMap::toROSData() const {
        std::vector<int8_t> data(map_params_.width*map_params_.height);
        float p;
        for (int i = 0; i < map_params_.width*map_params_.height; i++) {
            p = 1.0f/(1.0f + std::exp(-data_[i]));
            if (p > 0.7) {
                data[i] = 100;
            } else if (p < 0.3) {
                data[i] = 0;
            } else {
                data[i] = -1; 
            }
        }
        return data; 
    }

    MapParams OccupancyGridMap::getMapParams() const {
        return map_params_;
    }

    bool OccupancyGridMap::inBounds(int x, int y) const {
        return ((0<=x && x<map_params_.width) && (0<=y && y<map_params_.height));
    }

    void OccupancyGridMap::computeDistanceMap() {
        // Unknown will be given maximum dist value
        int width = map_params_.width;
        int height = map_params_.height;
        cv::Mat binary_map(height, width, CV_8UC1);
        for (int r = 0; r < height; r++) {
            for (int c = 0; c < width; c++) {
                // log odds > 2.33 ~ p > 0.7 ---> occupied
                binary_map.at<uchar>(r, c) = (data_[r*width+c] >= 2.33) ? 0 : 255; 
            }
        }
        cv::Mat dist_mat;
        cv::distanceTransform(binary_map, dist_mat, cv::DIST_L2, cv::DIST_MASK_5);
        double min_dist_grid_unit, max_dist_grid_unit;
        cv::minMaxLoc(dist_mat, &min_dist_grid_unit, &max_dist_grid_unit);
        max_dist_ = max_dist_grid_unit * map_params_.resolution;
        
        // Copy from opencv mat to flattened vector
        for (int r = 0; r < height; r++) {
            for (int c = 0; c < width; c++) {
                if (data_[r*width+c] == 0.0f) {
                    distance_map_[r*width+c] = max_dist_;
                } else {
                    distance_map_[r*width+c] = dist_mat.at<float>(r,c)*map_params_.resolution; 
                }
            }
        }
        distance_dirty_ = false; 
    }
}
