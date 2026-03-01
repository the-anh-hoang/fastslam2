#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp> 
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cstdint> 
#include <cmath> 
#include <chrono>
#include <thread>
#include <algorithm>

int WIDTH = 5000;
int HEIGHT = 5000; 
float MAP_RESOLUTION = 0.01; // m/cell
float  L0 = 0.0; 
std::vector<std::vector<float>> MAP(HEIGHT, std::vector<float>(WIDTH, L0));
float ALPHA = 2; // cell
float BETA  = 0.01; // rad
// robot in middle (0,0) world coordinates 
float ORIGIN_X = -WIDTH*MAP_RESOLUTION/2;
float ORIGIN_Y = -HEIGHT*MAP_RESOLUTION/2;

// Log-odds increments
const float L_OCC = 0.7;   // + 70%
const float L_FREE = -0.7;  // -60%
const float L_UNKNOWN = 0.0;

// Clamp limits to prevent overflow
const float L_MAX = 4.6;    // ~99% probability
const float L_MIN = -4.6;   // ~1% probability


class MapperNode : public rclcpp::Node {
    public:
    MapperNode() : Node("mapper_node")  {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&MapperNode::raycastingScanCallback, this, std::placeholders::_1)
        ); 
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map", 10
        );
        tf_buffer_ =  std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&MapperNode::publishMap, this)
        );  
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_; 
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr}; 
    rclcpp::TimerBase::SharedPtr timer_; 


    
    void raycastingScanCallback(const sensor_msgs::msg::LaserScan& scan) {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                "map", "base_scan", scan.header.stamp
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transfrom %s to %s: %s",
                "map", "base_scan", ex.what()
            );
        }
        double scan_x = t.transform.translation.x;
        double scan_y = t.transform.translation.y;
        auto [x_grid, y_grid] = worldToGridCoords(scan_x, scan_y);
        auto [x_grid_e, y_grid_e] = worldToGridCoordsExact(scan_x, scan_y);
        double scan_theta = getYaw(t);
        double ray_angle;
        

        for (int i = 0; i < scan.ranges.size(); i++) {
            if (scan.ranges[i] < scan.range_min || scan.ranges[i] > scan.range_max) continue; 
            int x_grid_curr = x_grid;
            int y_grid_curr = y_grid; 
            ray_angle = scan.angle_min + scan.angle_increment*i + scan_theta;
            double rdx = std::cos(ray_angle);
            double rdy = std::sin(ray_angle);
            double ro_fract_x = x_grid_e - x_grid;
            double ro_fract_y = y_grid_e - y_grid;
            double step_unit_x = std::sqrt(1+ (rdy/rdx)*(rdy/rdx));
            double step_unit_y = std::sqrt(1+ (rdx/rdy)*(rdx/rdy));
            
            double dist_x; 
            if (rdx > 0) {
                dist_x = (1-ro_fract_x)*step_unit_x;
            } else {
                dist_x = ro_fract_x*step_unit_x;
            }
            double dist_y;
            if (rdy > 0) {
                dist_y = (1-ro_fract_y)*step_unit_y;
            } else {
                dist_y = ro_fract_y*step_unit_y;
            }

            int step_dir_x = static_cast<int>(rdx / std::abs(rdx));
            int step_dir_y = static_cast<int>(rdy / std::abs(rdy));   
            double dist_traveled = 0; 
            double scan_range_grid = scan.ranges[i] / MAP_RESOLUTION; 
            while (dist_traveled < scan_range_grid && inBounds(x_grid_curr, y_grid_curr)) {
                MAP[y_grid_curr][x_grid_curr] = std::clamp(MAP[y_grid_curr][x_grid_curr] + L_FREE, L_MIN, L_MAX);
                if (dist_y < dist_x) {
                    y_grid_curr += step_dir_y;
                    dist_traveled = dist_y;
                    dist_y += step_unit_y;
                } else {
                    x_grid_curr += step_dir_x;
                    dist_traveled = dist_x; 
                    dist_x += step_unit_x; 
                }
            }
            
            if (inBounds(x_grid_curr, y_grid_curr)) {
                MAP[y_grid_curr][x_grid_curr] = std::clamp(MAP[y_grid_curr][x_grid_curr] + L_OCC, L_MIN, L_MAX);
                // occupancy threshold tolerance 
                for (int dy = -ALPHA; dy <= ALPHA; dy++) {
                    for (int dx = -ALPHA; dx <= ALPHA; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        if (inBounds(x_grid_curr+dx, y_grid_curr+dy)) {
                            MAP[y_grid_curr+dy][x_grid_curr+dx] = std::clamp(
                                MAP[y_grid_curr+dy][x_grid_curr+dx] + L_OCC, L_MIN, L_MAX);
                        }
                    }
                }
            }
        }
    }

    
    bool inBounds(int x, int y) {
        return ((0 <= x && x < WIDTH ) && (0<=y && y < HEIGHT));
    }
    

    void scanCallback(const sensor_msgs::msg::LaserScan& msg) {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                "map", "base_scan", tf2::TimePointZero
            );
        } catch (const tf2::TransformException &ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transfrom %s to %s: %s",
                "map", "base_scan", ex.what()
            );
        }
        double scan_x = t.transform.translation.x;
        double scan_y = t.transform.translation.y;
        double scan_theta = getYaw(t); 
        for (int y = 0; y < WIDTH; y++) {
            for (int x = 0; x < HEIGHT; x++) {
                auto [world_x, world_y] = gridToWorldCoords(x,y);
                MAP[y][x] += inverse_sensor_model(world_x, world_y, scan_x, scan_y, scan_theta, msg) - L0;
                
            }
        }
    }

    float inverse_sensor_model(double x, double y, double scan_x, double scan_y, double scan_theta,
        const sensor_msgs::msg::LaserScan& scan
    ) {
        double cell_to_scan_distance = std::sqrt((x-scan_x)*(x-scan_x) + (y-scan_y)*(y-scan_y));
        double cell_to_scan_ang = std::atan2(y-scan_y, x-scan_x) - scan_theta; // in robot frame
        while (cell_to_scan_ang > M_PI) cell_to_scan_ang -= 2*M_PI;
        while (cell_to_scan_ang < -M_PI) cell_to_scan_ang += 2*M_PI;
        // now we find the ray to this cell.
        int ray_idx = findBestRayIdx(cell_to_scan_ang, scan.angle_min, scan.angle_increment);
        double zt = scan.ranges[ray_idx]; 
        if (cell_to_scan_distance > zt 
            || (std::abs(cell_to_scan_ang-(scan.angle_min+ray_idx*scan.angle_increment))>BETA/2)) {
            // std::cout << cell_to_scan_distance << ", "<< scan.range_max << "\n"; 
            return L_UNKNOWN;
        }
        if ((zt < scan.range_max) && (std::abs(cell_to_scan_distance-zt) < ALPHA/2)) {
            // std::cout << "occ" << "\n"; 
            return L_OCC;

        }
        if (cell_to_scan_distance <= zt) {
            // std::cout << "free" << "\n";
            return L_FREE;
        }
        return L_UNKNOWN; 
    }
    
    int findBestRayIdx(double target_ang, double start_ang, double ang_increment) {
        return std::round((target_ang-start_ang)/ang_increment);
    }

    void publishMap() {
        auto map_msg = nav_msgs::msg::OccupancyGrid(); 
        auto header = std_msgs::msg::Header();
        auto map_meta_data = nav_msgs::msg::MapMetaData(); 

        header.stamp = this->now();
        header.frame_id = "map";
        map_meta_data.map_load_time.sec = 0;
        map_meta_data.map_load_time.nanosec = 0;
        map_meta_data.resolution = MAP_RESOLUTION;
        map_meta_data.width = WIDTH;
        map_meta_data.height = HEIGHT;
        map_meta_data.origin.position.x = ORIGIN_X;
        map_meta_data.origin.position.y = ORIGIN_Y;
        map_meta_data.origin.position.z = 0.0;
        map_meta_data.origin.orientation.x = 0; 
        map_meta_data.origin.orientation.y = 0; 
        map_meta_data.origin.orientation.z = 0; 
        map_meta_data.origin.orientation.w = 1;

        std::vector<std::int8_t> data(WIDTH*HEIGHT);
        for (int y =0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                float prob = 1.0f / (1.0f + std::exp(-MAP[y][x]));
                if (prob > 0.7) {
                    data[WIDTH*y + x] = 100; 
                } else if (prob < 0.3) {
                    data[WIDTH*y + x] = 0;
                } else {
                    data[WIDTH*y + x] = -1; 
                }
            }
        }
        map_msg.header = header;
        map_msg.info = map_meta_data;
        map_msg.data = data;
        map_pub_->publish(map_msg);




    }

    double getYaw(geometry_msgs::msg::TransformStamped& t) {
        double qx = t.transform.rotation.x;
        double qy = t.transform.rotation.y;
        double qz = t.transform.rotation.z;
        double qw = t.transform.rotation.w;
        return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    }
    

    // return cell x, y (not to be mistaken with r,c which is y x)
    std::pair<int, int> worldToGridCoords(double x, double y) {
        return {
            static_cast<int>((x-ORIGIN_X)/MAP_RESOLUTION),
            static_cast<int>((y-ORIGIN_Y)/MAP_RESOLUTION)
        };
    } 
    std::pair<double, double> worldToGridCoordsExact(double x, double y) {
        return {
            ((x-ORIGIN_X)/MAP_RESOLUTION),
            ((y-ORIGIN_Y)/MAP_RESOLUTION)
        };
    } 

    // return the world coordinates x,y in the center of that cell
    std::pair<double, double> gridToWorldCoords(int x, int y) {
        return {ORIGIN_X+(x+0.5)*MAP_RESOLUTION, ORIGIN_Y+(y+0.5)*MAP_RESOLUTION};
    }
    std::pair<double, double> gridEdgeToWorldCoords(int x, int y) {
        return {ORIGIN_X+x*MAP_RESOLUTION, ORIGIN_Y+y*MAP_RESOLUTION};
    }
    
    bool comp(double a, double b) {
        return a > b; 
    }



}; 

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapperNode>());
    rclcpp::shutdown();
    return 0; 
}