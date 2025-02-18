#ifndef MPCPlannerCorridors_HEADER
#define MPCPlannerCorridors_HEADER

#include "rclcpp/rclcpp.hpp"
#include <casadi/casadi.hpp>

class MPCPlannerCorridors: rclcpp::Node
{
public:
    MPCPlannerCorridors();

    void iniitializeModel();
    void setInputBounds();
    void setDynamicsConstraints();
    void setCost();
    void setInitialGuess();
    void solve();

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<custom_msgs_pkg::msg::PolygonArray>::SharedPtr polygon_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr travel_corridors_pub_;

    nav_msgs::msg::Odometry odometry_;
    vec_Vec2f obstacles_;
}
#endif