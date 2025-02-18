#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class DummyRiskMapPublisher : public rclcpp::Node {
public:
    DummyRiskMapPublisher() : Node("dummy_risk_map_publisher") {
        // Create the publisher
        risk_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/dummy_risk_map", 10);

        // Set the timer to publish at 1 Hz
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                        std::bind(&DummyRiskMapPublisher::publishRiskMap, this));

        RCLCPP_INFO(this->get_logger(), "Dummy risk map publisher node started.");
    }

private:
    void publishRiskMap() {
        nav_msgs::msg::OccupancyGrid risk_map;

        // Set metadata
        risk_map.header.stamp = this->get_clock()->now();
        risk_map.header.frame_id = "map";  // The coordinate frame

        // Grid properties
        risk_map.info.resolution = 0.2;  // 10 cm per cell
        risk_map.info.width = 100;  // 20x20 grid
        risk_map.info.height = 100;

        // Set the origin of the map (bottom-left corner in global coordinates)
        risk_map.info.origin.position.x = 0.0;
        risk_map.info.origin.position.y = 0.0;
        risk_map.info.origin.position.z = 0.0;
        risk_map.info.origin.orientation.w = 1.0;  // No rotation

        // Populate risk values (dummy data)
        risk_map.data.resize(risk_map.info.width * risk_map.info.height, -1); // Default -1 (unknown)

        for (size_t y = 0; y < risk_map.info.height; ++y) {
            for (size_t x = 0; x < risk_map.info.width; ++x) {
                size_t index = y * risk_map.info.width + x;
                
                // Example: Assign higher risk to the center area
                if (x > 3 && x < 12 && y > 8 && y < 10) {
                    risk_map.data[index] = 90;  // High risk
                } else {
                    risk_map.data[index] = 30;  // Low risk
                }
            }
        }

        // Publish the risk map
        risk_map_pub_->publish(risk_map);
        RCLCPP_INFO(this->get_logger(), "Published dummy risk map.");
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr risk_map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyRiskMapPublisher>());
    rclcpp::shutdown();
    return 0;
}
