#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
using std::placeholders::_1; 
class PointCloudHandler : public rclcpp::Node 
{
    public:
    PointCloudHandler() : Node("point_cloud_handler") {
        point_cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "point_cloud", 10, std::bind(&PointCloudHandler::point_cloud_callback, this, _1)
        );
    }

    private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub;

    void point_cloud_callback(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Points: %d", msg->width);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudHandler>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}