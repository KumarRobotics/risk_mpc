#include "rclcpp/rclcpp.hpp"
#include <casadi/casadi.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std;
using namespace casadi;

class TestRiskMap: public rclcpp::Node{
public:
    TestRiskMap(): Node("risk_map_test"){
        risk_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/dummy_risk_map", 10, bind(&TestRiskMap::riskMapCallback, this, placeholders::_1));
        risk_value_pub_ = this->create_publisher<std_msgs::msg::Float32>("/risk_value", 10);
        RCLCPP_INFO(this->get_logger(), "Test Risk Map Node initialized.");
    }

    void riskMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        test_risk_map_ = *msg;
        RCLCPP_INFO(this->get_logger(), "Value at 0 of data, %1d", test_risk_map_.data[0]);

        vector<double> risk_data_double(test_risk_map_.data.begin(), test_risk_map_.data.end());
        RCLCPP_INFO(this->get_logger(), "Value at 0 of risk_data_double, %1f", risk_data_double[0]);
        vector<casadi_int> grid_dims = {static_cast<casadi_int>(test_risk_map_.info.width), 
            static_cast<casadi_int>(test_risk_map_.info.height)};
        
        // bspline throws an error for some reasons??????
        continuous_risk_map_ = interpolant("continuous_risk_map", "linear", grid_dims, risk_data_double);
        
        // Query the interpolated risk map at (0,0)
        vector<DM> query_point{1.0, 1.0};
        vector<DM> result = continuous_risk_map_(query_point);
        RCLCPP_INFO(this->get_logger(), "result.size(), %1ld", result.size());
        RCLCPP_INFO(this->get_logger(), "result[0].get_str().c_str(), %s", result[0].get_str().c_str());
        auto risk_value = static_cast<float>(result[0].scalar());

        // Publish the risk value
        auto msg_out = std_msgs::msg::Float32();
        msg_out.data = risk_value;
        risk_value_pub_->publish(msg_out);
    }
private:
    nav_msgs::msg::OccupancyGrid test_risk_map_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr risk_map_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr risk_value_pub_;

    Function continuous_risk_map_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestRiskMap>());
    rclcpp::shutdown();
    return 0;
}
