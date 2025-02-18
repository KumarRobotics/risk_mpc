
#include <decomp_util/ellipsoid_decomp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <decomp_ros_msgs/msg/ellipsoid_array.hpp>
#include <decomp_ros_msgs/msg/polyhedron_array.hpp>
#include <decomp_ros_util/data_ros_utils.hpp>
#include <decomp_test_node/txt_reader.hpp>
#include <functional>
#include <memory>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

using namespace std::chrono_literals;

class TestPathDecomp3D : public rclcpp::Node {
public:
    TestPathDecomp3D() : Node("decomp_test_node") {
        this->declare_parameter("num_lat", num_lat_);
        this->declare_parameter("num_long", num_long_);
        this->declare_parameter("frame_id", frame_id_);

        num_lat_ = this->get_parameter("num_lat").as_int();
        num_long_ = this->get_parameter("num_long").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        testPathDecomp3D();

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud",
                                                                     10);
        timer_ = this->create_wall_timer(
                1000ms, std::bind(&TestPathDecomp3D::update, this));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path", 1);
        es_pub_ = this->create_publisher<decomp_ros_msgs::msg::EllipsoidArray>(
                "ellipsoid_array", 1);
        poly_pub_ = this->create_publisher<decomp_ros_msgs::msg::PolyhedronArray>(
                "polyhedron_array", 1);
    }

    ~TestPathDecomp3D() {}

private:
    void testPathDecomp3D() {
        const float radius = 5.0;
        for (int i = 0; i < num_lat_; ++i) {
            const float fr_lat = static_cast<float>(i) / static_cast<float>(num_lat_);
            const float phi = (fr_lat - 0.5) * M_PI;
            for (int j = 0; j < num_long_; ++j) {
                const float fr_long =
                        static_cast<float>(j) / static_cast<float>(num_long_);
                const float theta = fr_long * M_PI * 2.0;

                pcl::PointXYZRGB pt;
                pt = pcl::PointXYZRGB(50 + fr_lat * 205, 255 - fr_long * 100, 255);
                pt.x = radius * cos(phi) * cos(theta);
                pt.y = radius * cos(phi) * sin(theta);
                pt.z = radius * sin(phi);

                const uint8_t &pixel_r = 255 * fr_lat;
                const uint8_t &pixel_g = 255 * (1.0 - fr_long);
                const uint8_t &pixel_b = 255;
                // Define point color
                uint32_t rgb = (static_cast<uint32_t>(pixel_r) << 16 |
                                static_cast<uint32_t>(pixel_g) << 8 |
                                static_cast<uint32_t>(pixel_b));
                pt.rgb = *reinterpret_cast<float *>(&rgb);

                cloud_.points.push_back(pt);
            }
        }

        pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(cloud_, *pc2_msg_);

        pcl::PointCloud<pcl::PointXYZRGB> cloud2;
        pcl::fromROSMsg(*pc2_msg_, cloud2);
        std::cout << cloud2.points.size() << "\n";

        pc2_msg_->header.frame_id = frame_id_;
    }

    void update() {
        if (!pc2_msg_) {
            return;
        }
        pc2_msg_->header.stamp = now();
        pub_->publish(*pc2_msg_);
        vec_Vec3f obs = DecompROS::cloud_to_vec(pc2_msg_);
        // Read path from txt
        vec_Vec3f path;

        path = {{1, 1, 0}, {2, 2, 0}, {4, 4, 0}};
        nav_msgs::msg::Path path_msg = DecompROS::vec_to_path(path);
        path_msg.header.frame_id = "map";
        path_pub_->publish(path_msg);

        // Using ellipsoid decomposition
        EllipsoidDecomp3D decomp_util;
        decomp_util.set_obs(obs);
        decomp_util.set_local_bbox(Vec3f(1, 2, 1));
        decomp_util.dilate(
                path);// Set max iteration number of 10, do fix the path

        // Publish visualization msgs
        decomp_ros_msgs::msg::EllipsoidArray es_msg =
                DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
        es_msg.header.frame_id = "map";
        es_pub_->publish(es_msg);

        decomp_ros_msgs::msg::PolyhedronArray poly_msg =
                DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
        poly_msg.header.frame_id = "map";
        poly_pub_->publish(poly_msg);

        // Convert to inequality constraints Ax < b
        auto polys = decomp_util.get_polyhedrons();
        for (size_t i = 0; i < path.size() - 1; i++) {
            const auto pt_inside = (path[i] + path[i + 1]) / 2;
            LinearConstraint3D cs(pt_inside, polys[i].hyperplanes());
            printf("i: %zu\n", i);
            std::cout << "A: " << cs.A() << std::endl;
            std::cout << "b: " << cs.b() << std::endl;
            std::cout << "point: " << path[i].transpose();
            if (cs.inside(path[i]))
                std::cout << " is inside!" << std::endl;
            else
                std::cout << " is outside!" << std::endl;

            std::cout << "point: " << path[i + 1].transpose();
            if (cs.inside(path[i + 1]))
                std::cout << " is inside!" << std::endl;
            else
                std::cout << " is outside!" << std::endl;
        }
    }
    int num_lat_ = 100;
    int num_long_ = 100;
    std::string frame_id_ = "map";

    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<decomp_ros_msgs::msg::EllipsoidArray>::SharedPtr es_pub_;
    rclcpp::Publisher<decomp_ros_msgs::msg::PolyhedronArray>::SharedPtr poly_pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::spin(std::make_shared<TestPathDecomp3D>());
    rclcpp::shutdown();
    return 0;
}
