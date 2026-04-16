
#include "particle_filter_localizer/particle_filter.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cstdint> 
#include <cmath>
#include <fstream>
Particle::Particle(double x, double y, double theta, double w) : x(x), y(y), theta(theta), w(w) {}
PrevState::PrevState(double x, double y, double theta) : x(x), y(y), theta(theta) {}

ParticleFilter::ParticleFilter(int num_particles, rclcpp::Node* node) 
    : previous_state_(0.0, 0.0, 0.0),
      engine_(rd_()),
      node_(node) {
    
    num_particles_ = num_particles;
    particles_.reserve(num_particles);
    
    // Create debug marker publisher
    debug_marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
        "particle_filter/debug_markers", 10);
    
    RCLCPP_INFO(node_->get_logger(), "ParticleFilter initialized with %d particles", 
                num_particles);
}

void ParticleFilter::initialize(const nav_msgs::msg::OccupancyGrid& map) {
    // ------ init particles ------
    // find free cells
    map_meta_data_ = map.info; 

    std::vector<std::pair<int, int>> free_cells;
    for (unsigned int r = 0; r < map_meta_data_.height; r++) {
        for (unsigned int c = 0; c < map_meta_data_.width; c++) {
            if (map.data[r*map_meta_data_.width + c] == 0) {
                free_cells.push_back({c,r}); // x, y
            }
        }
    }
    
    std::uniform_int_distribution<size_t> cell_dist(0, free_cells.size()-1);
    std::uniform_real_distribution<double> angle_dist(0, 2.0*M_PI);

    int random_cell_idx;
    std::pair<int, int> random_cell; 
    float res = map_meta_data_.resolution;
    double x;
    double y;
    double theta;
    double weight = 1.0/num_particles_; 
    for (int i = 0; i < num_particles_; i++) {
        random_cell_idx = cell_dist(engine_);
        random_cell = free_cells[random_cell_idx];
        x = map_meta_data_.origin.position.x + (random_cell.first + 0.5)*res;
        y = map_meta_data_.origin.position.y + (random_cell.second + 0.5)*res;
        theta = angle_dist(engine_);
        particles_.push_back(Particle(x,y,theta,weight)); 
    }
    // for (const auto& p : particles_) {
    //     std::cout << p.x << ", " << p.y << ", " << p.theta << ", " << p.w << std::endl; 
    // }

    // ------- init distance map ------
    distance_map_ = computeDistanceMap(map);

}

std::vector<std::vector<float>> ParticleFilter::computeDistanceMap(const nav_msgs::msg::OccupancyGrid& map) {
    double resolution = map_meta_data_.resolution;
    int width =  map_meta_data_.width;
    int height = map_meta_data_.height;
    cv::Mat binary(height, width, CV_8UC1);
    
    for (int i = 0; i < height*width; i++) {
        // NOTE!!!: This does not work with binary map (check == 100) 
        binary.at<uchar>(i/width,i%width) = (map.data[i] >= 70) ? 0 : 255;
    }
    cv::Mat dist_cv;
    cv::distanceTransform(binary, dist_cv, cv::DIST_L2, cv::DIST_MASK_5); // read as float by default
    double minDist, maxDist;
    cv::minMaxLoc(dist_cv, &minDist, &maxDist);

    // ---------------------- Viz --------------------------------
    // DEBUG: Print statistics
    // std::cout << "Distance range (in cells):\n";
    // std::cout << "  Min: " << minDist << "\n";
    // std::cout << "  Max: " << maxDist << "\n";

    // // Count how many cells have distance > 0
    // int non_zero = cv::countNonZero(dist_cv > 0);
    // std::cout << "  Free cells: " << non_zero << " / " << (height*width) << "\n";
    // cv::imwrite("/tmp/binary_input.png", binary);
    // std::cout << "Saved binary input to /tmp/binary_input.png\n";

    // cv::Mat normalized;
    // cv::normalize(dist_cv, normalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    
    // // Apply colormap
    // cv::Mat colored;
    // cv::applyColorMap(normalized, colored, cv::COLORMAP_JET);
    // cv::imwrite("/tmp/distance_field.png", colored);
    // std::cout << "Saved to /tmp/distance_field.png\n";
    // --------------------------------------------------------
    max_dist_ = maxDist*map_meta_data_.resolution; 
    std::vector<std::vector<float>> distance_field(height, std::vector<float>(width));
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (map.data[width*i + j] == -1) {
                distance_field[i][j] = -1;
            } else {
                distance_field[i][j] = dist_cv.at<float>(i,j) * resolution;
            }
        }
    }
    return distance_field;
} 


void ParticleFilter::predict(const nav_msgs::msg::Odometry& odom_msg) {
    // SIMPLE MOTION MODEL: UPDATE EVERY PARTICLE BASED ON POSE
    std::normal_distribution<double> noise(0, 0.03);
    double curr_x = odom_msg.pose.pose.position.x;
    double curr_y = odom_msg.pose.pose.position.y;
    double qx = odom_msg.pose.pose.orientation.x;
    double qy = odom_msg.pose.pose.orientation.y;
    double qz = odom_msg.pose.pose.orientation.z;
    double qw = odom_msg.pose.pose.orientation.w;

    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    // compute deltas (how much we moved since prev state)
    double delta_x = (curr_x - previous_state_.x) ;
    double delta_y = (curr_y - previous_state_.y) ;
    double delta_trans = std::sqrt(delta_x*delta_x + delta_y*delta_y); 
    double delta_theta = yaw - previous_state_.theta; 
    previous_state_.x = curr_x;
    previous_state_.y = curr_y;
    previous_state_.theta = yaw;

    // loop through each particles
    for (int i = 0; i < num_particles_; i++) {
        // PREDICTING BASED ON ODOMETRY
        // TODO: A BASIC APPROXIMATION OF SIMPLE STRAIGHT LINE MOTION MODEL
        particles_[i].x += delta_trans*cos(particles_[i].theta) + noise(engine_);
        particles_[i].y += delta_trans*sin(particles_[i].theta) + noise(engine_);
        particles_[i].theta += delta_theta + noise(engine_);
        // normalizing rotation angle
        while (particles_[i].theta > M_PI) particles_[i].theta -= 2.0 * M_PI;
        while (particles_[i].theta < -M_PI) particles_[i].theta += 2.0 * M_PI;
    }
}

void ParticleFilter::update(const sensor_msgs::msg::LaserScan& scan, const geometry_msgs::msg::TransformStamped& t) {
    // GOAL: update weight of each particle with p(zt|x[k]_t)
    // loop through each particle:
    //      q = 1 (to accumulate probability of every ray)
    //      loop through each measurement z[k]_t in the scan:
    //          if z[k]_t != z_max (we ignore non return measurements): 
    //              transform end point to world frame based on each particle pose
    //              get closest distance to obstacles
    //              q *= zhit * probability(dist, std_dev_hit) + (z_random/z_max)
    //      w_particle = q

    double z_hit = 0.90;
    double std_hit =0.2;
    double z_rand = 0.05;
    double p_rand = 1/scan.range_max;

    double dx = t.transform.translation.x;
    double dy = t.transform.translation.y;
    double d_theta = quatToYaw(
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w
    ); 
    double x_scan_endpoint;
    double y_scan_endpoint; 
    double x_base;
    double y_base;
    double x_world;
    double y_world;
    int x_world_cellcoord;
    int y_world_cellcoord;
    double dist;

    
    double total_weight = 0.0;
    double q;
    double pz;
    for (Particle& p : particles_) {
        q = 1.0;
        for (uint64_t k = 0; k < scan.ranges.size(); k+=2) {
            pz=0.0;
            if (scan.ranges[k] >= scan.range_min && scan.ranges[k] <= scan.range_max) {
                x_scan_endpoint = scan.ranges[k] * std::cos(scan.angle_min + scan.angle_increment*k);
                y_scan_endpoint = scan.ranges[k] * std::sin(scan.angle_min + scan.angle_increment*k);
                // to figure out where each ray lands in map,
                //  first figure out where ray land in particle coords
                x_base = dx + (x_scan_endpoint*std::cos(d_theta) - y_scan_endpoint*std::sin(d_theta));
                y_base = dy + (x_scan_endpoint*std::sin(d_theta) + y_scan_endpoint*std::cos(d_theta));
                // then we figure out where this is in the world based on particle's global pose
                x_world = p.x + (x_base*std::cos(p.theta) - y_base*std::sin(p.theta));
                y_world = p.y + (x_base*std::sin(p.theta) + y_base*std::cos(p.theta)); 

                // get dist to obstacle from dist map
                x_world_cellcoord = static_cast<int>((x_world-map_meta_data_.origin.position.x)/map_meta_data_.resolution);
                y_world_cellcoord = static_cast<int>((y_world-map_meta_data_.origin.position.y)/map_meta_data_.resolution); 
                if ((0 > x_world_cellcoord  || x_world_cellcoord >= (int)map_meta_data_.width)
                    || (0 > y_world_cellcoord || y_world_cellcoord >= (int)map_meta_data_.height)) dist = max_dist_; 
                else {
                    dist = distance_map_[y_world_cellcoord][x_world_cellcoord]; // indexing r, c
                }
                if (dist < 0) continue;
                //1/(std_hit*sqrt(2*M_PI)) * 
                pz += z_hit * (std::exp(-(dist*dist) / (2*(std_hit*std_hit))));
                pz += z_rand*p_rand;
                assert(pz <= 1.0);
                assert(pz >= 0.0); 
                q += pz*pz*pz; 
            }
        }
        p.w *= q;
        total_weight += p.w;
        if (!best_particle_) {best_particle_ = &p;}
        else {
            if (p.w > best_particle_->w) {
                best_particle_ = &p; 
            }
        }
    }
    publishBestParticle();
    resample(total_weight);
}   

void ParticleFilter::resample(double total_weight) {
    for (auto& p : particles_) p.w /= total_weight; // normalize 
    std::vector<double> cdf(num_particles_);
    std::vector<Particle> Xt;
    Xt.reserve(num_particles_); 
    cdf[0] = particles_[0].w;
    for (int i = 1; i < num_particles_; i++) {
        cdf[i] = cdf[i-1] + particles_[i].w;
    }
    // Systematic Resampler
    std::uniform_real_distribution<double> start_point_dist(0, 1.0/num_particles_);
    double start_point = start_point_dist(engine_);
     
    int curr_particle = 0; 
    double u; 
    std::normal_distribution<double> noise(0, 0.02);
    for (int i = 0; i < num_particles_; i++) {
        u = start_point + i*(1.0/num_particles_);
        
        while (u > cdf[curr_particle] && curr_particle < num_particles_ - 1) curr_particle++;
        Xt.push_back(particles_[curr_particle]);
        Xt.back().x += noise(engine_); 
        Xt.back().y += noise(engine_);  
        Xt.back().w = 1.0/num_particles_; 
    } 
    particles_ = Xt; 
}

std::vector<double> ParticleFilter::caclBaseToOdomTransform(const geometry_msgs::msg::TransformStamped& t) {
    // find best particle
    // find map->best_particle transform
    // find odom->base_footprint transform : this is param t

    // calc map->odom = (map->best_particle_) * (odom->base_footprint)^-1

    double map_to_base_x = best_particle_->x;
    double map_to_base_y = best_particle_->y;
    double map_to_base_orientation = best_particle_->theta; 
    double odom_to_base_x = t.transform.translation.x;
    double odom_to_base_y = t.transform.translation.y;
    double odom_to_base_orientation = quatToYaw(
        t.transform.rotation.x, 
        t.transform.rotation.y, 
        t.transform.rotation.z, 
        t.transform.rotation.w
    );
    // TODO: useless 
    // (odom->base_f)^-1:
    double c = std::cos(odom_to_base_orientation);
    double s = std::sin(odom_to_base_orientation);
    double base_to_odom_orientation = -odom_to_base_orientation;
    double base_to_odom_x = -(c*odom_to_base_x + s*odom_to_base_y);
    double base_to_odom_y = (s*odom_to_base_x - c*odom_to_base_y);

    // map -> odom
    c = std::cos(map_to_base_orientation);
    s = std::sin(map_to_base_orientation);
    double map_to_odom_x = map_to_base_x + c*base_to_odom_x - s*base_to_odom_y;
    double map_to_odom_y = map_to_base_y + s*base_to_odom_x + c*base_to_odom_y;
    double map_to_odom_orientation = map_to_base_orientation + base_to_odom_orientation;
    return {map_to_odom_x, map_to_odom_y, map_to_odom_orientation}; 
}


void ParticleFilter::publishDebugMarkers(const sensor_msgs::msg::LaserScan& scan,
                                         const geometry_msgs::msg::TransformStamped& t) {
    
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Extract transform
    double dx = t.transform.translation.x;
    double dy = t.transform.translation.y;
    
    double qx = t.transform.rotation.x;
    double qy = t.transform.rotation.y;
    double qz = t.transform.rotation.z;
    double qw = t.transform.rotation.w;
    double d_theta = std::atan2(2.0 * (qw * qz + qx * qy), 
                                1.0 - 2.0 * (qy * qy + qz * qz));
    
    // Use first particle for visualization
    // Find best particle
    int best_idx = 0;
    double max_weight = particles_[0].w;
    
    for (size_t i = 1; i < particles_.size(); i++) {
        if (particles_[i].w > max_weight) {
            max_weight = particles_[i].w;
            best_idx = i;
        }
    }
    
    const Particle& p = particles_[best_idx];
    
    int marker_id = 0;
    
    // Visualize scan endpoints (subsample for performance)
    for (size_t k = 0; k < scan.ranges.size(); k += 20) {  // Every 20th ray
        
        if (scan.ranges[k] < scan.range_min || 
            scan.ranges[k] >= scan.range_max) {
            continue;
        }
        
        // Transform endpoint to world frame
        double angle = scan.angle_min + k * scan.angle_increment;
        double x_scan = scan.ranges[k] * std::cos(angle);
        double y_scan = scan.ranges[k] * std::sin(angle);
        
        double x_base = dx + (x_scan * std::cos(d_theta) - y_scan * std::sin(d_theta));
        double y_base = dy + (x_scan * std::sin(d_theta) + y_scan * std::cos(d_theta));
        
        double x_world = p.x + (x_base * std::cos(p.theta) - y_base * std::sin(p.theta));
        double y_world = p.y + (x_base * std::sin(p.theta) + y_base * std::cos(p.theta));
        
        // Check if in bounds
        int x_cell = static_cast<int>((x_world - map_meta_data_.origin.position.x) 
                                      / map_meta_data_.resolution);
        int y_cell = static_cast<int>((y_world - map_meta_data_.origin.position.y) 
                                      / map_meta_data_.resolution);
        
        bool in_bounds = (x_cell >= 0 && x_cell < (int)map_meta_data_.width &&
                         y_cell >= 0 && y_cell < (int)map_meta_data_.height);
        
        // Create marker for this endpoint
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = node_->now();
        marker.ns = "scan_endpoints";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = x_world;
        marker.pose.position.y = y_world;
        marker.pose.position.z = 0.0;
        
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        
        // Color based on bounds
        if (in_bounds) {
            double dist = distance_map_[y_cell][x_cell];
            
            // Color by distance: green = far from obstacles, red = near obstacles
            if (dist < 0.1) {
                marker.color.r = 1.0;  // Red - very close
                marker.color.g = 0.0;
                marker.color.b = 0.0;
            } else if (dist < 0.5) {
                marker.color.r = 1.0;  // Orange - close
                marker.color.g = 0.5;
                marker.color.b = 0.0;
            } else {
                marker.color.r = 0.0;  // Green - far
                marker.color.g = 1.0;
                marker.color.b = 0.0;
            }
        } else {
            // Out of bounds - blue
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;
        }
        
        marker.color.a = 0.8;
        marker.lifetime = rclcpp::Duration::from_seconds(0.5);  // Fade after 0.5s
        
        marker_array.markers.push_back(marker);
    }
    
    // Add particle pose as arrow
    visualization_msgs::msg::Marker particle_marker;
    particle_marker.header.frame_id = "map";
    particle_marker.header.stamp = node_->now();
    particle_marker.ns = "particle_pose";
    particle_marker.id = marker_id++;
    particle_marker.type = visualization_msgs::msg::Marker::ARROW;
    particle_marker.action = visualization_msgs::msg::Marker::ADD;
    
    particle_marker.pose.position.x = p.x;
    particle_marker.pose.position.y = p.y;
    particle_marker.pose.position.z = 0.0;
    
    // Convert theta to quaternion
    double cy = std::cos(p.theta * 0.5);
    double sy = std::sin(p.theta * 0.5);
    particle_marker.pose.orientation.w = cy;
    particle_marker.pose.orientation.x = 0.0;
    particle_marker.pose.orientation.y = 0.0;
    particle_marker.pose.orientation.z = sy;
    
    particle_marker.scale.x = 0.3;  // Arrow length
    particle_marker.scale.y = 0.05; // Arrow width
    particle_marker.scale.z = 0.05; // Arrow height
    
    particle_marker.color.r = 1.0;
    particle_marker.color.g = 1.0;
    particle_marker.color.b = 0.0;
    particle_marker.color.a = 1.0;
    
    particle_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    
    marker_array.markers.push_back(particle_marker);
    
    // Publish all markers
    debug_marker_pub_->publish(marker_array);
}

void ParticleFilter::clearDebugMarkers() {
    visualization_msgs::msg::MarkerArray marker_array;
    
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = "map";
    delete_marker.header.stamp = node_->now();
    delete_marker.ns = "scan_endpoints";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    
    marker_array.markers.push_back(delete_marker);
    
    delete_marker.ns = "particle_pose";
    marker_array.markers.push_back(delete_marker);
    
    debug_marker_pub_->publish(marker_array);
}


void ParticleFilter::publishBestParticle() {
    if (particles_.empty()) return;
    


    
    const Particle& best = *best_particle_;
    
    
    // Create marker
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.ns = "best_particle";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    marker.pose.position.x = best.x;
    marker.pose.position.y = best.y;
    marker.pose.position.z = 0.2;
    
    // Quaternion
    double cy = std::cos(best.theta * 0.5);
    double sy = std::sin(best.theta * 0.5);
    marker.pose.orientation.w = cy;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = sy;
    
    // BIG!
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    // Magenta
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    marker.lifetime = rclcpp::Duration::from_seconds(1.0);
    
    visualization_msgs::msg::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    
    debug_marker_pub_->publish(marker_array); 
    
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "Best particle: (%.2f, %.2f, %.2f°) w=%.2e",
                        best.x, best.y, best.theta * 180.0 / M_PI, best.w);
}

double ParticleFilter::quatToYaw(double qx, double qy, double qz, double qw) {
    return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}