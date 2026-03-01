#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <rclcpp/rclcpp.hpp>

#include <vector>
#include <random>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>        
#include <visualization_msgs/msg/marker_array.hpp>

struct PrevState{
    double x;
    double y;
    double theta;
    PrevState(double x, double y, double theta); 
};

struct Particle {
    double x;
    double y;
    double theta;
    double w;
    Particle(double x, double y, double theta, double w);
    Particle(); 
};

class ParticleFilter {
    public:
        ParticleFilter(int num_particles, rclcpp::Node* node);

        void initialize(const nav_msgs::msg::OccupancyGrid& map);
        
        const std::vector<Particle>& getParticles() const {
        return particles_;
        }

        void predict(const nav_msgs::msg::Odometry& odom_msg);

        void update(const sensor_msgs::msg::LaserScan& scan, const geometry_msgs::msg::TransformStamped& t); 
        
        void resample(double total_weight); 

        std::vector<double> caclBaseToOdomTransform(const geometry_msgs::msg::TransformStamped& t); 

        void publishDebugMarkers(
            const sensor_msgs::msg::LaserScan& scan,
            const geometry_msgs::msg::TransformStamped& t
        );
        void clearDebugMarkers();
        void publishBestParticle();

    private:
        int num_particles_;
        nav_msgs::msg::MapMetaData map_meta_data_; 
        std::vector<Particle> particles_;

        Particle* best_particle_; 
        PrevState previous_state_;
        std::random_device rd_;
        std::mt19937 engine_;
        std::vector<std::vector<float>> distance_map_;
        double quatToYaw(double qx, double qy, double qz, double qw);
        std::vector<std::vector<float>> computeDistanceMap(const nav_msgs::msg::OccupancyGrid& map); 
        double max_dist_; 
        // Debug visualization
        rclcpp::Node* node_;                                                    
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr 
        debug_marker_pub_;
};
#endif 