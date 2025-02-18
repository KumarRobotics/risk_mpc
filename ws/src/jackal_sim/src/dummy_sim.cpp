#include <random>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "std_srvs/srv/empty.hpp"
#include "jackal_sim/srv/set_goal.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class DummySim : public rclcpp::Node {
public:
    DummySim() : Node("dummy_sim") {
        occupancy_grid_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/cost_map", 10);
        goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        current_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/current_pose", 10);

        timer_occupancy_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DummySim::publish_occupancy_grid, this));
        timer_pose_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&DummySim::publish_current_pose, this));

        control_subscription_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("/cmd_vel", 1, std::bind(&DummySim::move_jackal, this, _1));
        
        goal_pose_service_ = this->create_service<jackal_sim::srv::SetGoal>(
            "/set_goal_pose", std::bind(&DummySim::publish_goal_pose_service_callback, this, _1, _2)
        );

        initialize_parameters();
        RCLCPP_INFO(this->get_logger(), "Made Dummy Jackal Node");
    }

private:
    void initialize_parameters(){
        // set the current pose
        current_pose_.header.frame_id = "map";
        current_pose_.header.stamp = this->get_clock()->now();

        // Set current pose
        current_pose_.pose.position.x = -0.1;
        current_pose_.pose.position.y = -0.1;
        current_pose_.pose.orientation.w = 1.0;

        // Grid metadata
        grid_.header.frame_id = "map";
        grid_.info.resolution = resolution_;
        grid_.info.width = width_;
        grid_.info.height = height_;
        grid_.info.origin.position.x = x_min_;
        grid_.info.origin.position.y = y_min_;
        grid_.info.origin.orientation.z = 0.0;
        grid_.info.origin.orientation.w = 1.0;
    }

    void move_jackal(const geometry_msgs::msg::TwistStamped & msg){
        // determine the appropriate dt
        rclcpp::Time current_time = msg.header.stamp;
        if (last_cmd_vel_time_.nanoseconds() > 0) {
            dt_ = (current_time - last_cmd_vel_time_).seconds();

            // in case the commands stall
            if (dt_ > 0.5) {
                dt_ = 0.0;
                RCLCPP_WARN(this->get_logger(), "Stall detected: Time between updates exceeded 0.5 seconds.");
            }
        } else {
            dt_ = 0.0;
        }

        last_cmd_vel_time_ = current_time;
        
        double current_x = current_pose_.pose.position.x;
        double current_y = current_pose_.pose.position.y;
        double current_roll, current_pitch, current_yaw;
        tf2::Quaternion current_quat(current_pose_.pose.orientation.x, 
                                    current_pose_.pose.orientation.y, 
                                    current_pose_.pose.orientation.z, 
                                    current_pose_.pose.orientation.w);
        tf2::Matrix3x3(current_quat).getRPY(current_roll, current_pitch, current_yaw);
        
        double vel = msg.twist.linear.x;
        double omega = msg.twist.angular.z;

        // add some noise to the jackal motion
        std::normal_distribution<double> linear_noise(0, vel*dt_/10);
        double new_x = current_x + vel*std::cos(current_yaw)*dt_ + linear_noise(motion_generator_);
        double new_y = current_y + vel*std::sin(current_yaw)*dt_ + linear_noise(motion_generator_);

        // calculate the new quaternion
        tf2::Quaternion delta_quat;
        // noise to the rotation
        std::normal_distribution<double> angular_noise(0, omega*dt_/10);
        delta_quat.setRPY(0.0, 0.0, omega*dt_ + angular_noise(motion_generator_));
        tf2::Quaternion new_quat = current_quat * delta_quat;
        new_quat.normalize();
        current_pose_.header.stamp = this->get_clock()->now();
        current_pose_.pose.position.x = new_x;
        current_pose_.pose.position.y = new_y;
        current_pose_.pose.orientation.x = new_quat.x();
        current_pose_.pose.orientation.y = new_quat.y();
        current_pose_.pose.orientation.z = new_quat.z();
        current_pose_.pose.orientation.w = new_quat.w();
    }

    void publish_occupancy_grid() {
        // Random occupancy grid data
        std::default_random_engine grid_generator_(13); 

        std::vector<int8_t> grid_data(width_* height_, 0);
        std::uniform_real_distribution<double> cluster_dist_width(x_min_, x_max_);
        std::uniform_real_distribution<double> cluster_dist_height(y_min_, y_max_);
        std::normal_distribution<double> point_dist(0.0, 0.2);

        // generate 10 clusters
        for (int i = 0; i < 10; ++i) {
            double center_x = cluster_dist_width(grid_generator_);
            double center_y = cluster_dist_height(grid_generator_);

            // generate 40 points per cluster
            for (int j = 0; j < 40; ++j) {
                double point_x = center_x + point_dist(grid_generator_);
                double point_y = center_y + point_dist(grid_generator_);

                int grid_x = std::clamp(static_cast<int>((point_x - x_min_) / resolution_), 0, static_cast<int>(width_ - 1));
                int grid_y = std::clamp(static_cast<int>((point_y - y_min_) / resolution_), 0, static_cast<int>(height_ - 1));
                grid_data[grid_x * width_ + grid_y] = 100;
            }
        }

        grid_.data = grid_data;
        grid_.header.stamp = this->get_clock()->now();
        occupancy_grid_publisher_->publish(grid_);
    }

    void publish_goal_pose_service_callback(
        const std::shared_ptr<jackal_sim::srv::SetGoal::Request> request,
        std::shared_ptr<jackal_sim::srv::SetGoal::Response> response){
        geometry_msgs::msg::PoseStamped goal_pose = geometry_msgs::msg::PoseStamped();
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->get_clock()->now();

        // Set goal pose
        goal_pose.pose.position.x = request->pose.position.x;
        goal_pose.pose.position.y = request->pose.position.y;
        goal_pose.pose.orientation.z = request->pose.orientation.z;
        goal_pose.pose.orientation.w = request->pose.orientation.w;

        goal_pose_publisher_->publish(goal_pose);
        response->success = true;
        RCLCPP_INFO(this->get_logger(), "Published goal position: (%f, %f)", goal_pose.pose.position.x, goal_pose.pose.position.y);
    }


    void publish_current_pose() {
        current_pose_publisher_->publish(current_pose_);
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_publisher_;

    rclcpp::TimerBase::SharedPtr timer_occupancy_;
    rclcpp::TimerBase::SharedPtr timer_pose_;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr control_subscription_;

    rclcpp::Service<jackal_sim::srv::SetGoal>::SharedPtr goal_pose_service_;

    // occupancy grid parameters
    nav_msgs::msg::OccupancyGrid grid_ = nav_msgs::msg::OccupancyGrid();
    int height_ = 100;
    int width_ = 100;
    double resolution_ = 0.1;
    double x_max_ = 5.0;
    double x_min_ = -5.0;
    double y_max_ = 5.0;
    double y_min_ = -5.0;

    // jackal sim parameters
    double dt_ = 0.0;
    rclcpp::Time last_cmd_vel_time_;
    geometry_msgs::msg::PoseStamped current_pose_;

    // noise generators
    std::default_random_engine motion_generator_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummySim>());
    rclcpp::shutdown();
    return 0;
}